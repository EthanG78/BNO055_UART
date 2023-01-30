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

    bno055_offsets_t calOffsets;
    if (bno_get_calibration_data(&calOffsets) == -1)
    {
        return 1;
    }


    if (bno_close() == -1)
    {
        return 1;
    }


    return 0;
}