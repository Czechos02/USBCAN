# Makefile

TARGET = usbcan
BUILD_DIR = build
SRC = usbcan.cpp
INC = usbcan.h
LIB = lib$(TARGET).a

CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17 -O2 -I.

$(BUILD_DIR)/$(LIB): $(SRC) $(INC) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $(SRC) -o $(BUILD_DIR)/$(TARGET).o
	ar rcs $(BUILD_DIR)/$(LIB) $(BUILD_DIR)/$(TARGET).o

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

.PHONY: clean install uninstall

clean:
	rm -rf $(BUILD_DIR)

install: $(BUILD_DIR)/$(LIB)
	# Copy library to system library path
	sudo cp $(BUILD_DIR)/$(LIB) /usr/local/lib/
	sudo cp $(INC) /usr/local/include/usbcan
	sudo ldconfig

uninstall:
	sudo rm -f /usr/local/lib/$(LIB)
	sudo rm -f /usr/local/include/usbcan
	sudo ldconfig
