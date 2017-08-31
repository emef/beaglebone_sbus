#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include "../sbus.h"

int main(int argc, char** argv) {
    sbus_t* sbus = sbus_new(2, 10, SBUS_CONFIG_PINS | SBUS_NONBLOCKING);
    if (!sbus) {
        fprintf(stderr, "Could not create SBUS handle: %s\n", strerror(errno));
        return 1;
    }

    uint16_t channels[16] = { 0 };

    // initialize some values in the "write" channels
    for (int i=0; i<16; i++) {
        channels[i] = 100 * (i + 1);
    }

    while (1) {
        if (0 != sbus_write(sbus, channels)) {
            fprintf(stderr, "Could not write to SBUS handle, %s\n", strerror(errno));
            return 1;
        }


        for (int i=0; i<16; i++) {
            channels[i]++;
        }

        sleep(10);
    }
}
