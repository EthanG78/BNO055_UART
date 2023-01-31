/*
 * bno055_uart_cal_restore_test.c
 * Test getting calibration offsets from BNO055
 * sensors and restoring to their saved state.
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
    fprintf(stdout, "Current status:\n\tMagnetometer: %d\n\tAccelerometer: %d\n\tGyroscope: %d\n\tSystem: %d", cal[3], cal[2], cal[1], cal[0]);

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

            fprintf(stdout, "Current status:\n\tMagnetometer: %d\n\tAccelerometer: %d\n\tGyroscope: %d\n\tSystem: %d", cal[3], cal[2], cal[1], cal[0]);

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

    // Get the calibration offsets and store within
    // a bno055_offsets_t struct.
    bno055_offsets_t calOffsets;
    if (bno_get_calibration_data(&calOffsets) == -1)
    {
        return 1;
    }

    fprintf(stdout, "\nAccel Offsets:\n\tX: %d ", offsets->accel_offset_x);
    fprintf(stdout, "\n\tY: %d ", offsets->accel_offset_y);
    fprintf(stdout, "\n\tZ: %d ", offsets->accel_offset_z);
    fprintf(stdout, "\nMag Offsets:\n\tX: %d ", offsets->mag_offset_x);
    fprintf(stdout, "\n\tY: %d ", offsets->mag_offset_y);
    fprintf(stdout, "\n\tZ: %d ", offsets->mag_offset_z);
    fprintf(stdout, "\nGyro Offsets:\n\tX: %d ", offsets->gyro_offset_x);
    fprintf(stdout, "\n\tY: %d ", offsets->gyro_offset_y);
    fprintf(stdout, "\n\tZ: %d ", offsets->gyro_offset_z);
    fprintf(stdout, "\nAccel Radius: %d ", offsets->accel_radius);
    fprintf(stdout, "\nMag Radius: %d\n", offsets->mag_radius);


    if (bno_set_calibration(&calOffsets) == -1)
    {
        return 1;
    }

    if (bno_close() == -1)
    {
        return 1;
    }

    return 0;
}