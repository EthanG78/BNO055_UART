/*
 * bno055_uart_euler_test.c
 * Test reading abolsute orientation
 * as euler angles from the BNO055.
 * Author: Ethan Garnier
 */
#include <stdio.h>
#include <bno055_uart.h>

int main()
{
    if (bno_init("/dev/serial0", OPERATION_MODE_NDOF) == -1)
    {
        return 1;
    }

    // Read vector data and printout
    // absolute orientation along with
    // calibration status
    bno055_vector_t euler;
    uint8_t cal[4];
    while (1)
    {
        if (bno_read_euler(&euler) == -1)
            break;

        if (bno_get_calibration_status(cal) == -1)
            break;

        fprintf(stdout, "Heading=%.2f Roll=%.2f Pitch=%.2f\tSys_cal=%d Gyro_cal=%d  Accel_cal=%d  Mag_cal=%d\n",
                euler.heading, euler.roll, euler.pitch, cal[0], cal[1], cal[2], cal[3]);

        delay(1000);
    }

    if (bno_close() == -1)
    {
        return 1;
    }

    return 0;
}