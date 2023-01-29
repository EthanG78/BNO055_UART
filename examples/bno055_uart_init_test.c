/*
 * bno055_uart_init_test.c
 * Test initialization code of bno055_uart.
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

    fprintf(stdout, "SUCCESSFULLY INITIALIZED BNO055\n");

    if (bno_close() == -1)
    {
        return 1;
    }

    fprintf(stdout, "SUCCESSFULLY CLOSED BNO055 COMMUNICATIONS\n");

    return 0;
}