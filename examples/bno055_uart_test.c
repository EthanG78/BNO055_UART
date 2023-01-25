/*
 * bno055_uart_test.c
 * Small test for bno055_uart library
 * Author: Ethan Garnier
 */
#include <stdio.h>
#include <bno055_uart.h>

int main()
{
    if (int bno_init("/dev/serial0", OPERATION_MODE_NDOF) == -1)
    {
        return 1
    }

    fprintf(stdout, "SUCCESSFULLY INITIALIZED BNO055\n");

    return 0;
}