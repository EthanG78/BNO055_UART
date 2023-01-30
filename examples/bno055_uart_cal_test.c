/*
 * bno055_uart_cal_test.c
 * Test getting calibration status
 * from BNO055 and calibrating the device.
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

    uint8_t cal[4];
    if (bno_get_calibration_status(cal) == -1)
    {
        fprintf(stderr, "Error fetching calibration status registers\n");
        return 1;
    }

    fprintf(stdout, "\n-----------\n");
    fprintf(stdout, "Calibrating BNO055 Device:\n");

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

    if (bno_close() == -1)
    {
        return 1;
    }

    return 0;
}