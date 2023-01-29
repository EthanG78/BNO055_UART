/*
 * bno055_uart_system_test.c
 * Test get_revision and get_system_status function
 * of the bno055_uart library.
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

    uint8_t rev[6];
    if (bno_get_revision(rev) == -1)
    {
        fprintf(stderr, "Failed to get revision information from bno055\n");
        return 1;
    }

    fprintf(stdout, "BNO055 Revision Information:");
    uint16_t sw = ((rev[1] << 8) | rev[0]) & 0xFFFF;
    fprintf(stdout, "\n\tSoftware revision: 0x%02x%02x", sw);
    fprintf(stdout, "\n\tBootloader version: 0x%02x", rev[2]);
    fprintf(stdout, "\n\tAccelerometer ID: 0x%02x", rev[3]);
    fprintf(stdout, "\n\tMagnetometer ID: 0x%02x", rev[4]);
    fprintf(stdout, "\n\tGyro ID: 0x%02x\n\n", rev[5]);


    uint8_t status[3];
    if (get_system_status(status, 1) == -1)
    {
        fprintf(stderr, "Failed to get system status information from bno055\n");
        return 1;
    }

    fprintf(stdout, "BNO055 System Status Information:");
    fprintf(stdout, "\n\tSystem status register: 0x%02x", status[0]);
    fprintf(stdout, "\n\tSystem error register: 0x%02x", status[1]);
    fprintf(stdout, "\n\tSelf test result register: 0x%02x", status[2]);

    if (bno_close() == -1)
    {
        return 1;
    }

    return 0;
}