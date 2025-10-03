#include "usbcan.h"
#include <fcntl.h>
#include <asm/termbits.h>
#include <stdio.h>
#include <string.h>
#include <cerrno>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <iostream>


/*************************************************************************
* @brief USBCAN constructor
**************************************************************************/
USBCAN::USBCAN(std::string tty_dev, CANUSB_SPEED speed,
                CANUSB_MODE mode, CANUSB_FRAME frame,
                CANUSB_PAYLOAD_MODE payload_mode,
                int baudrate)
{
    _tty_dev = tty_dev;
    _speed = speed;
    _mode = mode;
    _frame = frame;
    _payload_mode = payload_mode;
    _baudrate = baudrate;

    _tty_fd = adapterInit();

    commandSettings();
    _program_running = true;
    reception_callback = nullptr;

    rx_thread = nullptr;
    mutex = new std::mutex;
    data = new unsigned char[8];
    rx_message = new CAN_MESSAGE;
    rx_message->ID = 0;
    rx_message->data = data;
    rx_message->length = 0;

}

/*************************************************************************
* @brief USBCAN destructor
**************************************************************************/
USBCAN::~USBCAN()
{
  _program_running = false;
  if (rx_thread && rx_thread->joinable()) rx_thread->join();
  if (_tty_fd != -1) close(_tty_fd);
  if(rx_thread) delete rx_thread;
  delete mutex;
  delete[] data;
  delete rx_message;
}

/*************************************************************************
* @brief USBCAN transmit data to CAN
**************************************************************************/
int USBCAN::sendData(CAN_MESSAGE* tx_message)
{

  unsigned char binary_id_lsb = 0, binary_id_msb = 0;

  int error = 0;

  if (tx_message->length == 0) {
    fprintf(stderr, "Data length should be larger than 0!\n");
    return -1;
  }

  binary_id_msb = ((tx_message->ID & 0xFF00) >> 8); 
  binary_id_lsb = (tx_message->ID & 0x00FF);
  std::lock_guard<std::mutex> lock(*mutex);
  error = sendDataFrame(binary_id_lsb, binary_id_msb, tx_message->data, tx_message->length);

  return error;
}

/*************************************************************************
* @brief USBCAN start listening for data from CAN and call callback
**************************************************************************/
void USBCAN::startListening()
{
    rx_thread = new std::thread(&USBCAN::receiveData, this);
}

/*************************************************************************
* @brief USBCAN register own callback for data reception
**************************************************************************/
void USBCAN::registerCallback(void (*callback_function)(CAN_MESSAGE* rx_message))
{
  reception_callback = callback_function;
}

/*=====================================================================================================
=======================================================================================================
=========================== PRIVATE METHODS OF USBCAN CLASS ===========================================
=======================================================================================================
=======================================================================================================*/

/*************************************************************************
* @brief USBCAN receive data from CAN
**************************************************************************/
void USBCAN::receiveData()
{
  int frame_len;
  unsigned char frame[32];
  int msb_id, lsb_id;

  while(_program_running)
  {
  frame_len = frameRecv(frame, sizeof(frame));

  if (frame_len == -1) 
  {
    printf("Frame recieve error!\n");

  }
  else
  {
    if ((frame_len >= 6) && (frame[0] == 0xaa) && ((frame[1] >> 4) == 0xc))
    {
        msb_id = frame[3];
        lsb_id = frame[2];
        rx_message->ID = ((msb_id<<8) | lsb_id);
        memcpy(rx_message->data,frame+4,frame_len-5);
        rx_message->length = frame_len-5;
        if(reception_callback)
        {
          (*reception_callback)(rx_message);
        }
    }

  }
  }

}

/*************************************************************************
* @brief USBCAN init Waveshare USB-CAN-A converter
**************************************************************************/
int USBCAN::adapterInit()
{
    int tty_fd, result;
    struct termios2 tio;

    tty_fd = open(_tty_dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (tty_fd == -1) {
        fprintf(stderr, "open(%s) failed: %s\n", _tty_dev.c_str(), strerror(errno));
        return -1;
    }

    result = ioctl(tty_fd, TCGETS2, &tio);
    if (result == -1) {
        fprintf(stderr, "ioctl() failed: %s\n", strerror(errno));
        close(tty_fd);
        return -1;
    }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag = BOTHER | CS8 | CSTOPB;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_ispeed = _baudrate;
    tio.c_ospeed = _baudrate;

    result = ioctl(tty_fd, TCSETS2, &tio);
    if (result == -1) {
        fprintf(stderr, "ioctl() failed: %s\n", strerror(errno));
        close(tty_fd);
        return -1;
    }

  return tty_fd;
}

/*************************************************************************
* @brief USBCAN set command settings in USB-CAN-A converter
**************************************************************************/
int USBCAN::commandSettings()
{
    int cmd_frame_len;
    unsigned char cmd_frame[20];

    cmd_frame_len = 0;
    cmd_frame[cmd_frame_len++] = 0xaa;
    cmd_frame[cmd_frame_len++] = 0x55;
    cmd_frame[cmd_frame_len++] = 0x12;
    cmd_frame[cmd_frame_len++] = _speed;
    cmd_frame[cmd_frame_len++] = _frame;
    cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
    cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
    cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
    cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
    cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
    cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
    cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
    cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
    cmd_frame[cmd_frame_len++] = _mode;
    cmd_frame[cmd_frame_len++] = 0x01;
    cmd_frame[cmd_frame_len++] = 0;
    cmd_frame[cmd_frame_len++] = 0;
    cmd_frame[cmd_frame_len++] = 0;
    cmd_frame[cmd_frame_len++] = 0;
    cmd_frame[cmd_frame_len++] = generateChecksum(&cmd_frame[2], 17);

    if (frameSend(cmd_frame, cmd_frame_len) < 0) {
        return -1;
    }

    return 0;
}

/*************************************************************************
* @brief USBCAN generate checksum
**************************************************************************/
int USBCAN::generateChecksum(const unsigned char *data, int data_len)
{
  int checksum;

  checksum = 0;
  for (int i = 0; i < data_len; i++) {
    checksum += data[i];
  }

  return checksum & 0xff;
}

/*************************************************************************
* @brief USBCAN send given frame to CAN
**************************************************************************/
int USBCAN::frameSend(const unsigned char *frame, int frame_len)
{
    int i;

    printf(">>> ");
    for (i = 0; i < frame_len; i++) {
      printf("%02x ", frame[i]);
    }
    
    printf("\n");


  int result;
  result = write(_tty_fd, frame, frame_len);
  if (result == -1) {
    fprintf(stderr, "write() failed: %s\n", strerror(errno));
    return -1;
  }

  return frame_len;
}

/*************************************************************************
* @brief USBCAN receive frame from CAN
**************************************************************************/
int USBCAN::frameRecv(unsigned char *frame, int frame_len_max)
{
  int result, frame_len, checksum;
  unsigned char byte;

  frame_len = 0;
  while (_program_running) {

    {
    std::lock_guard<std::mutex> lock(*mutex);
    result = read(_tty_fd, &byte, 1);
    }
    if (result == -1) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        fprintf(stderr, "read() failed: %s\n", strerror(errno));
        return -1;
      }

    } else if (result > 0) {

      if (frame_len == frame_len_max) {
        fprintf(stderr, "frameRecv() failed: Overflow\n");
        return -1;
      }
      
      frame[frame_len++] = byte;

      if (frameIsComplete(frame, frame_len)) {
        break;
      }
    }

    usleep(10);
  }

  /* Compare checksum for command frames only. */
  if ((frame_len == 20) && (frame[0] == 0xaa) && (frame[1] == 0x55)) {
    checksum = generateChecksum(&frame[2], 17);
    if (checksum != frame[frame_len - 1]) {
      fprintf(stderr, "frameRecv() failed: Checksum incorrect\n");
      return -1;
    }
  }

  return frame_len;
}

/*************************************************************************
* @brief USBCAN check if frame is complete
**************************************************************************/
int USBCAN::frameIsComplete(const unsigned char *frame, int frame_len)
{
  if (frame_len > 0) {
    if (frame[0] != 0xaa) {
      /* Need to sync on 0xaa at start of frames, so just skip. */
      return 1;
    }
  }

  if (frame_len < 2) {
    return 0;
  }

  if (frame[1] == 0x55) { /* Command frame... */
    if (frame_len >= 20) { /* ...always 20 bytes. */
      return 1;
    } else {
      return 0;
    }
  } else if ((frame[1] >> 4) == 0xc) { /* Data frame... */
    if (frame_len >= (frame[1] & 0xf) + 5) { /* ...payload and 5 bytes. */
      return 1;
    } else {
      return 0;
    }
  }

  /* Unhandled frame type. */
  return 1;
}

/*************************************************************************
* @brief USBCAN create whole CAN frame and send it to CAN magistral
**************************************************************************/
int USBCAN::sendDataFrame(unsigned char id_lsb, unsigned char id_msb, unsigned char data[], int data_length_code)
{
#define MAX_FRAME_SIZE 13
  int data_frame_len = 0;
  unsigned char data_frame[MAX_FRAME_SIZE] = {0x00};

  if (data_length_code < 0 || data_length_code > 8)
  {
    fprintf(stderr, "Data length code (DLC) must be between 0 and 8!\n");
    return -1;
  }

  /* Byte 0: Packet Start */
  data_frame[data_frame_len++] = 0xaa;

  /* Byte 1: CAN Bus Data Frame Information */
  data_frame[data_frame_len] = 0x00;
  data_frame[data_frame_len] |= 0xC0; /* Bit 7 Always 1, Bit 6 Always 1 */
  if (_frame == CANUSB_FRAME_STANDARD)
    data_frame[data_frame_len] &= 0xDF; /* STD frame */
  else /* CANUSB_FRAME_EXTENDED */
    data_frame[data_frame_len] |= 0x20; /* EXT frame */
  data_frame[data_frame_len] &= 0xEF; /* 0=Data */
  data_frame[data_frame_len] |= data_length_code; /* DLC=data_len */
  data_frame_len++;

  /* Byte 2 to 3: ID */
  data_frame[data_frame_len++] = id_lsb; /* lsb */
  data_frame[data_frame_len++] = id_msb; /* msb */

  /* Byte 4 to (4+data_len): Data */
  for (int i = 0; i < data_length_code; i++)
    data_frame[data_frame_len++] = data[i];

  /* Last byte: End of frame */
  data_frame[data_frame_len++] = 0x55;

  if (frameSend(data_frame, data_frame_len) < 0)
  {
    fprintf(stderr, "Unable to send frame!\n");
    return -1;
  }

  return 0;
}