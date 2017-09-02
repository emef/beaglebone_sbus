#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include "../sbus.h"

int main(int argc, char** argv) {
    sbus_t* sbus = sbus_new(1, 10, SBUS_CONFIG_PINS );
    if (!sbus) {
        fprintf(stderr, "Could not create SBUS handle: %s\n", strerror(errno));
        return 1;
    }

    int channel = argc > 1 ? atoi(argv[1]) : 0;

    uint16_t channels[16] = { 0 };

    for (int loops=0; ; loops++) {
        if (0 != sbus_read(sbus, channels)) {
            if (errno != EAGAIN) {
                fprintf(stderr, "Could not read from SBUS handle, %s\n", strerror(errno));
                return 1;
            }
        } else {
            if (loops % 10 == 0) {
                printf("channel[%d] = %d       lost frames = %d/%d %.2f%%\n",
                       channel, channels[channel], sbus_lost_frames(sbus), loops,
                       ((float) sbus_lost_frames(sbus)) / loops);
            }
        }
    }
}
