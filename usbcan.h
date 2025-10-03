#ifndef LL_SYSTEM_CAN_CAN_H
#define LL_SYSTEM_CAN_CAN_H

#pragma once

#include <string>
#include <atomic>
#include <thread>
#include <mutex>

typedef enum {
  CANUSB_SPEED_1000000 = 0x01,
  CANUSB_SPEED_800000  = 0x02,
  CANUSB_SPEED_500000  = 0x03,
  CANUSB_SPEED_400000  = 0x04,
  CANUSB_SPEED_250000  = 0x05,
  CANUSB_SPEED_200000  = 0x06,
  CANUSB_SPEED_125000  = 0x07,
  CANUSB_SPEED_100000  = 0x08,
  CANUSB_SPEED_50000   = 0x09,
  CANUSB_SPEED_20000   = 0x0a,
  CANUSB_SPEED_10000   = 0x0b,
  CANUSB_SPEED_5000    = 0x0c,
} CANUSB_SPEED;

typedef enum {
  CANUSB_MODE_NORMAL          = 0x00,
  CANUSB_MODE_LOOPBACK        = 0x01,
  CANUSB_MODE_SILENT          = 0x02,
  CANUSB_MODE_LOOPBACK_SILENT = 0x03,
} CANUSB_MODE;

typedef enum {
  CANUSB_FRAME_STANDARD = 0x01,
  CANUSB_FRAME_EXTENDED = 0x02,
} CANUSB_FRAME;

typedef enum {
  CANUSB_INJECT_PAYLOAD_MODE_RANDOM      = 0,
  CANUSB_INJECT_PAYLOAD_MODE_INCREMENTAL = 1,
  CANUSB_INJECT_PAYLOAD_MODE_FIXED       = 2,
} CANUSB_PAYLOAD_MODE;


typedef struct _CAN_MESSAGE
{
  unsigned char*  data;
  unsigned int    length;
  int    ID;
} CAN_MESSAGE;

class USBCAN
{
    public:
        USBCAN(std::string tty_dev,
                CANUSB_SPEED speed,
                CANUSB_MODE mode,
                CANUSB_FRAME frame,
                CANUSB_PAYLOAD_MODE payload_mode,
                int baudrate = 2000000);
        ~USBCAN();
        void startListening();
        void registerCallback(void (*callback_function)(CAN_MESSAGE* rx_message));
        int sendData(CAN_MESSAGE* tx_message);

    private:
        int adapterInit();
        int commandSettings();
        void receiveData();

        int generateChecksum(const unsigned char *data, int data_len);
        int frameSend(const unsigned char *frame, int frame_len);
        int frameRecv(unsigned char *frame, int frame_len_max);
        int frameIsComplete(const unsigned char *frame, int frame_len);
        int sendDataFrame(unsigned char id_msb, unsigned char id_lsb, unsigned char data[], int data_length_code);

        std::string _tty_dev;
        CANUSB_SPEED _speed;
        CANUSB_MODE _mode;
        CANUSB_FRAME _frame;
        CANUSB_PAYLOAD_MODE _payload_mode;
        int _baudrate;
        int _tty_fd;
        std::atomic<bool> _program_running;
        int _terminate_after;
        float _inject_sleep_gap;
        void (*reception_callback)(CAN_MESSAGE* rx_message);
        std::thread *rx_thread;
        std::mutex *mutex;
        CAN_MESSAGE *rx_message;
        unsigned char* data;

};


#endif //LL_SYSTEM_CAN_CAN_H