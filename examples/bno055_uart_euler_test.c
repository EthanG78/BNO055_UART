/*
 * bno055_uart_read_vector_test.c
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

    // Calibrate the sensors first
    uint8_t cal[4];
    if (bno_get_calibration_status(cal) == -1)
    {
        fprintf(stderr, "Error fetching calibration status registers\n");
        return 1;
    }

    fprintf(stdout, "\n-----------\n");
    fprintf(stdout, "Calibrating BNO055 Device:\n");
    fprintf(stdout, "Current status:\n\tMagnetometer: %d\n\tAccelerometer: %d\n\tGyroscope: %d\n\tSystem: %d\n", cal[3], cal[2], cal[1], cal[0]);

    for (int i = 3; i >= 0; i--)
    {
        switch (i)
        {
        case 3:
            fprintf(stdout, "\tTo calibrate the Magnetometer, move the device through the air in a figure-8 pattern...\n");
            break;
        case 2:
            fprintf(stdout, "\tTo calibrate the Accelerometer, rotate the device around an axis in 45 degree increments...\n");
            break;
        case 1:
            fprintf(stdout, "\tTo calibrate the Gyroscope, place the device down and let rest still...\n");
            break;
        case 0:
            fprintf(stdout, "\tCalibrating system\n");
            break;
        }

        // Loop until that specific calibration register reads fully calibrated
        while (cal[i] != 3)
        {
            if (bno_get_calibration_status(cal) == -1)
            {
                fprintf(stderr, "Error fetching calibration status registers\n");
                return 1;
            }

            fprintf(stdout, "Current status:\n\tMagnetometer: %d\n\tAccelerometer: %d\n\tGyroscope: %d\n\tSystem: %d\n", cal[3], cal[2], cal[1], cal[0]);

            delay(1000);
        }

        switch (i)
        {
        case 3:
            fprintf(stdout, "Done calibrating magnetometer\n");
            break;
        case 2:
            fprintf(stdout, "Done calibrating accelerometer\n");
            break;
        case 1:
            fprintf(stdout, "Done calibrating gyroscope\n");
            break;
        case 0:
            fprintf(stdout, "Done calibrating system\n");
            break;
        }
    }

    // Read vector data and printout
    // absolute orientation
    bno055_vector_t euler;
    while (1)
    {
        if (bno_read_euler(&euler) == -1)
        {
            break;
        }

        fprintf(stdout, "\nHeading: %.2f\n", euler.heading);
        fprintf(stdout, "Roll: %.2f\n", euler.roll);
        fprintf(stdout, "Pitch: %.2f\n", euler.pitch);

        delay(500);
    }

    if (bno_close() == -1)
    {
        return 1;
    }

    return 0;
}