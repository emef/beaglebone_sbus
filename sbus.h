/*
author: Matt Forbes <matt.r.forbes@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#ifndef BB_SBUS_H
#define BB_SBUS_H

#include <fcntl.h>
#include <linux/serial.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

// --------------------------------------------------
// Interface

// Open SBUS flags
#define SBUS_CONFIG_PINS 0x1
#define SBUS_NONBLOCKING 0x2

typedef struct _sbus_t sbus_t;

/*
  Creates a new SBUS handler on specified uart (/dev/ttyOX).

  Valid flags are:

    SBUS_CONFIG_PINS - If set, will call `config-pin` to set up the
                       associated pins as uart.

    SBUS_NONBLOCKING - By default, reads to SBUS will be blocking until
                       a full packet has been received. If SBUS_NONBLOCKING
                       is set, reads will be non-blocking.

  Returns:
    An SBUS handler which can be used to read/write to the underlying
    uart device.
 */
sbus_t* sbus_new(int uart_no, int timeout_ms, uint8_t flags);

/*
  Attempt to read from the SBUS serial connection. On success,
  channels data will be written to `channels_out`.

  Returns:
    0 for success

    -1 for failure - errno will be set to EAGAIN when a full packet was
                     not available.
 */
int sbus_read(sbus_t* sbus, uint16_t* channels_out);

/*
  Write channels data to the SBUS serial connection.

  Returns:
    0 for success, -1 for failure.
 */
int sbus_write(sbus_t* sbus, uint16_t* channels_in);

// --------------------------------------------------
// Implementation

// SBUS protocol constants
#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00
#define SBUS2_FOOTER 0x04
#define SBUS_BAUD_RATE 100000

struct _sbus_t {
    int fd;
    uint8_t buffer[25];
    int buffer_ix;
    int timeout_ms;
    struct timespec packet_start;
};

// Helper forward declarations.
int config_pins(int uart_no);
int open_uart(int uart_no, bool blocking);
int decode_packet(uint8_t* packet, uint16_t* channels_out);

sbus_t* sbus_new(int uart_no, int timeout_ms, uint8_t flags) {
    if (flags & SBUS_CONFIG_PINS) {
        if (0 != config_pins(uart_no)) {
            return NULL;
        }
    }

    bool blocking = !(flags & SBUS_NONBLOCKING);

    int fd = open_uart(uart_no, blocking);
    if (fd < 0) {
        return NULL;
    }

    sbus_t* sbus = (sbus_t*) malloc(sizeof(sbus_t));
    if (!sbus) {
        return NULL;
    }

    sbus->fd = fd;
    sbus->buffer_ix = 0;
    sbus->timeout_ms = timeout_ms;
    memset(sbus->buffer, 0, 25);
    clock_gettime(CLOCK_MONOTONIC_RAW, &sbus->packet_start);

    return sbus;
}

int sbus_read(sbus_t *sbus, uint16_t* channels_out) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC_RAW, &now);
    uint64_t delta_ms = (now.tv_sec - sbus->packet_start.tv_sec) * 1000 +
        (now.tv_nsec - sbus->packet_start.tv_nsec) / 1000000;

    // If packet has timed out, just reset the buffer.
    if (delta_ms > sbus->timeout_ms) {
        sbus->buffer_ix = 0;
    }

    int to_read = 25 - sbus->buffer_ix;
    ssize_t count = read(sbus->fd, sbus->buffer + sbus->buffer_ix, to_read);
    if (count <= 0) {
        errno = count == 0 ? EAGAIN : errno;
        return -1;
    }

    if (sbus->buffer_ix == 0) {
        // search for the HEADER byte.
        for (int i=0; i<count; i++) {
            if (sbus->buffer[i] == SBUS_HEADER) {
                sbus->buffer_ix = 25 - i;

                if (i != 0) {
                    // header found part-way through buffer, slide everything to left.
                    memmove(sbus->buffer, sbus->buffer + i, 25 - i);
                }

                clock_gettime(CLOCK_MONOTONIC_RAW, &sbus->packet_start);

                break;
            }
        }
    } else {
        sbus->buffer_ix += count;
    }

    if (sbus->buffer_ix == 25) {
        // we have a full packet, now we must decode it.
        sbus->buffer_ix = 0;

        // ensure footer is valid, if not we throw away the packet.
        uint8_t footer = sbus->buffer[24];
        if (footer == SBUS_FOOTER || (footer & 0x0F) == SBUS2_FOOTER) {
            return decode_packet(sbus->buffer + 1, channels_out);
        }

    } else if (count == to_read) {
        // we don't have a full packet but there may still be more to read
        return sbus_read(sbus, channels_out);
    }

    // didn't get a full, valid packet
    errno = EAGAIN;
    return -1;
}

int decode_packet(uint8_t* packet, uint16_t* channels_out) {
    channels_out[0] = (int16_t) ((packet[0] | packet[1]<<8) & 0x07FF);
    channels_out[1] = (int16_t) ((packet[1]>>3 |packet[2]<<5) & 0x07FF);
    channels_out[2] = (int16_t) ((packet[2]>>6 |packet[3]<<2 |packet[4]<<10) & 0x07FF);
    channels_out[3] = (int16_t) ((packet[4]>>1 |packet[5]<<7) & 0x07FF);
    channels_out[4] = (int16_t) ((packet[5]>>4 |packet[6]<<4) & 0x07FF);
    channels_out[5] = (int16_t) ((packet[6]>>7 |packet[7]<<1 |packet[8]<<9) & 0x07FF);
    channels_out[6] = (int16_t) ((packet[8]>>2 |packet[9]<<6) & 0x07FF);
    channels_out[7] = (int16_t) ((packet[9]>>5 |packet[10]<<3) & 0x07FF);
    channels_out[8] = (int16_t) ((packet[11]   |packet[12]<<8) & 0x07FF);
    channels_out[9] = (int16_t) ((packet[12]>>3|packet[13]<<5) & 0x07FF);
    channels_out[10] = (int16_t) ((packet[13]>>6|packet[14]<<2|packet[15]<<10) & 0x07FF);
    channels_out[11] = (int16_t) ((packet[15]>>1|packet[16]<<7) & 0x07FF);
    channels_out[12] = (int16_t) ((packet[16]>>4|packet[17]<<4) & 0x07FF);
    channels_out[13] = (int16_t) ((packet[17]>>7|packet[18]<<1|packet[19]<<9) & 0x07FF);
    channels_out[14] = (int16_t) ((packet[19]>>2|packet[20]<<6) & 0x07FF);
    channels_out[15] = (int16_t) ((packet[20]>>5|packet[21]<<3) & 0x07FF);

    return 0;
}

int sbus_write(sbus_t* sbus, uint16_t* channels_in) {
    uint8_t packet[25];

    // SBUS header
    packet[0] = SBUS_HEADER;

    // 16 channels_in of 11 bit data
  	packet[1] = (uint8_t) ((channels_in[0] & 0x07FF));
  	packet[2] = (uint8_t) ((channels_in[0] & 0x07FF)>>8 | (channels_in[1] & 0x07FF)<<3);
  	packet[3] = (uint8_t) ((channels_in[1] & 0x07FF)>>5 | (channels_in[2] & 0x07FF)<<6);
  	packet[4] = (uint8_t) ((channels_in[2] & 0x07FF)>>2);
  	packet[5] = (uint8_t) ((channels_in[2] & 0x07FF)>>10 | (channels_in[3] & 0x07FF)<<1);
  	packet[6] = (uint8_t) ((channels_in[3] & 0x07FF)>>7 | (channels_in[4] & 0x07FF)<<4);
  	packet[7] = (uint8_t) ((channels_in[4] & 0x07FF)>>4 | (channels_in[5] & 0x07FF)<<7);
  	packet[8] = (uint8_t) ((channels_in[5] & 0x07FF)>>1);
  	packet[9] = (uint8_t) ((channels_in[5] & 0x07FF)>>9 | (channels_in[6] & 0x07FF)<<2);
  	packet[10] = (uint8_t) ((channels_in[6] & 0x07FF)>>6 | (channels_in[7] & 0x07FF)<<5);
  	packet[11] = (uint8_t) ((channels_in[7] & 0x07FF)>>3);
  	packet[12] = (uint8_t) ((channels_in[8] & 0x07FF));
  	packet[13] = (uint8_t) ((channels_in[8] & 0x07FF)>>8 | (channels_in[9] & 0x07FF)<<3);
  	packet[14] = (uint8_t) ((channels_in[9] & 0x07FF)>>5 | (channels_in[10] & 0x07FF)<<6);
  	packet[15] = (uint8_t) ((channels_in[10] & 0x07FF)>>2);
  	packet[16] = (uint8_t) ((channels_in[10] & 0x07FF)>>10 | (channels_in[11] & 0x07FF)<<1);
  	packet[17] = (uint8_t) ((channels_in[11] & 0x07FF)>>7 | (channels_in[12] & 0x07FF)<<4);
  	packet[18] = (uint8_t) ((channels_in[12] & 0x07FF)>>4 | (channels_in[13] & 0x07FF)<<7);
  	packet[19] = (uint8_t) ((channels_in[13] & 0x07FF)>>1);
  	packet[20] = (uint8_t) ((channels_in[13] & 0x07FF)>>9 | (channels_in[14] & 0x07FF)<<2);
  	packet[21] = (uint8_t) ((channels_in[14] & 0x07FF)>>6 | (channels_in[15] & 0x07FF)<<5);
  	packet[22] = (uint8_t) ((channels_in[15] & 0x07FF)>>3);

  	// flags
    packet[23] = 0x00;

    // footer
    packet[24] = SBUS_FOOTER;

    if (write(sbus->fd, packet, 25) != 25) {
        return -1;
    }

    return 0;
}

int config_pins(int uart_no) {
    char rx_enable[256], tx_enable[256];
    switch (uart_no) {
    case 1:
        strcpy(rx_enable, "config-pin P9_26 uart");
        strcpy(tx_enable, "config-pin P9_24 uart");
        break;

    case 2:
        strcpy(rx_enable, "config-pin P9_22 uart");
        strcpy(tx_enable, "config-pin P9_21 uart");
        break;

    case 4:
        strcpy(rx_enable, "config-pin P9_11 uart");
        strcpy(tx_enable, "config-pin P9_13 uart");
        break;

    case 5:
        strcpy(rx_enable, "config-pin P8_38 uart");
        strcpy(tx_enable, "config-pin P8_37 uart");
        break;

    default:
        fprintf(stderr, "Unsupported uart\n");
        return -1;
    }

    if (0 != system(rx_enable)) {
        fprintf(stderr, "Error configuring rx pin as uart");
        return -1;
    }

    if (0 != system(tx_enable)) {
        fprintf(stderr, "Error configuring tx pin as uart");
        return -1;
    }

    return 0;
}

int open_uart(int uart_no, bool blocking) {
    char tty_dev[256];
    snprintf(tty_dev, 256, "/dev/ttyO%d", uart_no);
    int fd = open(tty_dev, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        return -1;
    }

    struct termios options;
    struct serial_struct serinfo;
    int rate = SBUS_BAUD_RATE;

    serinfo.reserved_char[0] = 0;
    if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0) {
        return -1;
    }

    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    serinfo.custom_divisor = (serinfo.baud_base + (rate / 2)) / rate;
    if (serinfo.custom_divisor < 1) {
        serinfo.custom_divisor = 1;
    }
    if (ioctl(fd, TIOCSSERIAL, &serinfo) < 0) {
        return -1;
    }
    if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0) {
        return -1;
    }
    if (serinfo.custom_divisor * rate != serinfo.baud_base) {
        fprintf(stderr, "actual baudrate is %d / %d = %f",
              serinfo.baud_base, serinfo.custom_divisor,
              (float)serinfo.baud_base / serinfo.custom_divisor);
    }

    fcntl(fd, F_SETFL, 0);
    tcgetattr(fd, &options);
    cfsetispeed(&options, B38400);
    cfsetospeed(&options, B38400);
    cfmakeraw(&options);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CRTSCTS;
    options.c_cc[VMIN] = blocking ? 1 : 0;
    options.c_cc[VTIME] = 1;
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        return -1;
    }

    return fd;
}

#endif
