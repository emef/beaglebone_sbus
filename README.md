Library to read/write SBUS on BeagleBone.

Example usage:

```c
#include "sbus.h"

...

// Open SBUS connection on beaglebone UART1 with a 10ms timeout in non-blocking mode.
// Specifying SBUS_CONFIG_PINS will trigger calls to `config-pin` required to
// setup UART1 pins in uart mode.
sbus_t* sbus = sbus_new(1, 10, SBUS_CONFIG_PINS | SBUS_NONBLOCKING);
if (!sbus) {
    fprintf(stderr, "Could not create SBUS handle: %s\n", strerror(errno));
    return 1;
}

uint16_t channels[16] = { 0 };

while (1) {
    if (0 != sbus_read(sbus, channels)) {
        if (errno != EAGAIN) {
            fprintf(stderr, "Could not read from SBUS handle, %s\n", strerror(errno));
            return 1;
        }
    } else {
        for (int i=0; i<16; i++) {
            printf("channel[%d] = %d\n", i, channels[i]);
        }
    }

    sleep(1);
}
```

References:
  * https://developer.mbed.org/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/
  * https://github.com/bolderflight/SBUS
