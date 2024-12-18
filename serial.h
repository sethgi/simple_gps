#ifndef GPS_PROJECT_SERIAL_H
#define GPS_PROJECT_SERIAL_H

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


int open_serial_port(const char* device) {
  int serial_port = open(device, O_RDONLY | O_NOCTTY);
  if (serial_port < 0) {
    perror("Error opening serial port");
    return -1;
  }

  struct termios options;
  tcgetattr(serial_port, &options);
  cfsetispeed(&options, B115200); // Set baud rate
  cfsetospeed(&options, B115200);
  options.c_cflag |= (CLOCAL | CREAD); // Enable receiver
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8; // 8 data bits
  options.c_cflag &= ~PARENB; // No parity
  options.c_cflag &= ~CSTOPB; // 1 stop bit

  tcsetattr(serial_port, TCSANOW, &options);
  
  return serial_port;
}

int open_file(const char* fname) {
  return open(fname, O_RDONLY);
}

char* read_serial_port(const int port) {
  char buffer[1024];
  size_t index = 0;

  while (1) {
    char byte;
    int num_bytes = read(port, &byte, 1);
    if (num_bytes < 0) {
      perror("Error reading from serial port");
      return NULL;
    } else if (num_bytes == 0) {
      continue; // No data, wait for more
    }

    if (byte == '$') {
      index = 0; // Start of a new NMEA sentence
    }

    buffer[index++] = byte;

    if (byte == '\n' && index > 1) {
      buffer[index] = '\0'; // Null-terminate the string
      return strdup(buffer);
    }

    if (index >= sizeof(buffer) - 1) {
      index = 0; // Reset if buffer is full without a newline
    }
  }
}

#endif
