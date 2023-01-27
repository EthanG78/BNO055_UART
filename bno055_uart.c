/*
 * bno055_uart.c
 * BNO055_UART C Driver
 * Author: Ethan Garnier
 */
#include "bno055_uart.h"

// TODO: Look into moving the i/o helpers to other file

// Send the command cmd of size nCmdBytes to the BNO055 and wait for a response if
// ack is true. Store the response in byte array resp of size nRespBytes. If the BNO055
// indicates a bus error, resend the command MAX_CMD_SEND_ATTEMPTS times.
//
// Return 1 on success, -1 on error.
int send_serial_cmd(uint8_t *cmd, int nCmdBytes, uint8_t *resp, bool ack)
{
    int nAttempts = 0;
    while (1)
    {
        // Flush the serial port
        serialFlush(serial_fp);

        // Send our command over the serial port
        if (write(serial_fp, cmd, nCmdBytes) == -1)
        {
            perror("send_serial_cmd() send error:");
            return -1;
        }

        // If we don't want an ack, then return
        if (!ack)
            return 1;

        // Wait until serial data is available
        while (!serialDataAvail(serial_fp))
        {
        }

        // TODO: WE NEED TO HAVE THIS WAIT FOR DATA
        // Otherwise, look for an acknowledgment
        if (read(serial_fp, resp, 2) == -1)
        {
            perror("send_serial_cmd() timeout waiting for ack:");
            return -1;
        }

        // If there is no bus error, return
        if (!(resp[0] == 0xEE && resp[1] == 0x07))
            return 1;

        // If there was a bus error, retry
        nAttempts += 1;
        if (nAttempts > MAX_CMD_SEND_ATTEMPTS)
        {
            fprintf(stderr, "Exceeded maximum attempts for acknowledgment\n");
            return -1;
        }
    }
}

// Write byte array to register with address addr on BNO055. If ack
// is true, then expect an acknowledgment from the device.
//
// Return 1 on success, -1 on error.
int write_bytes(bno055_register_t addr, uint8_t *bytes, uint8_t nBytes, bool ack)
{
    // Build the write command with the following format:
    // Byte 1: Start Byte 0xAA
    // Byte 2: Write Command 0x00
    // Byte 3: Register Address
    // Byte 4: Number of bytes to be written
    // Byte 5 -> n + 4: Bytes to be written
    int cmdSize = 4 + nBytes;
    uint8_t cmd[cmdSize];

    cmd[0] = 0xAA;
    cmd[1] = 0x00;
    cmd[2] = addr & 0xFF;
    cmd[3] = nBytes & 0xFF;
    for (int i = 0; i < nBytes; i++)
    {
        cmd[4 + i] = bytes[i] & 0xFF;
    }

    fprintf(stdout, "First 5 bytes of cmd: 0x%02x%02x%02x%02x%02x\n", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

    // Send write command over serial, only allow for
    // 5 attempts (ignoring bus errors)
    uint8_t resp[2];
    if (send_serial_cmd(cmd, cmdSize, resp, ack) == -1)
    {
        fprintf(stderr, "Error sending serial command\n");
        return -1;
    }

    // Check to make sure we received a valid response
    // if we are expecting an acknowledgment
    if (resp == NULL || (ack && resp[0] != 0xEE && resp[1] != 0x01))
    {
        fprintf(stderr, "Failed to write to register: 0x%02x%02x\n", resp[0], resp[1]);
        return -1;
    }

    return 1;
}

// Write a single byte to register with address addr on BNO055. If ack
// is true, then expect an acknowledgment from the device.
//
// Return 1 on success, -1 on error.
int write_byte(bno055_register_t addr, uint8_t byte, bool ack)
{
    uint8_t bytes[] = {byte & 0xFF};
    if (write_bytes(addr, bytes, 0x01, ack) == -1)
        return -1;

    return 1;
}

// Read nBytes from register with address addr and store
// in byte array bytes.
//
// Return 1 on success, -1 on error.
int read_bytes(bno055_register_t addr, uint8_t *bytes, uint8_t nBytes)
{
    // Build the read command with the following format:
    // Byte 1: Start Byte 0xAA
    // Byte 2: Read Command 0x01
    // Byte 3: Register Address
    // Byte 4: Number of bytes to be read
    uint8_t cmd[] = {0xAA,
                     0x01,
                     addr & 0xFF,
                     nBytes & 0xFF};

    fprintf(stdout, "First 4 bytes of cmd: 0x%02x%02x%02x%02x\n", cmd[0], cmd[1], cmd[2], cmd[3]);

    // Send read command over serial, only allow for
    // 5 attempts (ignoring bus errors)
    uint8_t resp[2];
    if (send_serial_cmd(cmd, 4, resp, true) == -1)
    {
        fprintf(stderr, "Error sending serial command\n");
        return -1;
    }

    // Process the response we received
    if (resp[0] != 0xBB)
    {
        fprintf(stderr, "Failed to read register: 0x%02x%02x\n", resp[0], resp[1]);
        return -1;
    }

    // Debug
    fprintf(stderr, "Response in read bytes: 0x%02x%02x\n", resp[0], resp[1]);

    // Check to make sure we get the right number of bytes returned
    if (resp[1] != nBytes)
    {
        fprintf(stderr, "Returned data length different from expected:\n\tExpected: %d, Returned: %d\n", (int)nBytes, (int)resp[1]);
        return -1;
    }

    // Read the bytes we requested
    if (read(serial_fp, bytes, (int)nBytes) == -1)
    {
        perror("read_bytes() unable to read bytes:");
        return -1;
    }

    return 1;
}

// Read a single byte from register with address
// addr and return it.
//
// Return requested byte on success, NULL on failure (is this bad?)
uint8_t read_byte(bno055_register_t addr)
{
    uint8_t readByte[1];
    if (read_bytes(addr, readByte, 0x01) == -1)
    {
        fprintf(stderr, "Error reading single byte\n");
    }

    fprintf(stdout, "Byte read: 0x%02x\n", readByte[0]);

    // Ew no error checking
    return readByte[0];
}

// Sets the operation mode of the BNO055 based
// on the op modes defined in the bno055_opmode_t enum.
// Please refer to the BNO055 datasheet for descriptions
// on each of these operation modes.
//
// Return 1 on success, -1 on error.
int bno_set_mode(bno055_opmode_t mode)
{
    int success = write_byte(BNO055_OPR_MODE_ADDR, mode & 0xFF, true);

    // Sleep 30 miliseconds
    struct timespec ts;
    ts.tv_sec = 30 / 1000;
    ts.tv_nsec = (30 % 1000) * 1000000;
    nanosleep(&ts, NULL);

    if (success == -1)
    {
        fprintf(stderr, "Error changing operating mode of BNO055\n");
        return -1;
    }

    return 1;
}

// Initialize communication with a BNO055 IMU over serial
// port specified by serialPort. This MUST BE CALLED BEFORE
// ANY OTHER FUNCTION IN THIS LIBRARY.
//
// Return 1 on success, -1 on error.
int bno_init(char *serialPort, bno055_opmode_t mode)
{
    // Open serial port
    serial_fp = serialOpen(serialPort, 115200);
    if (serial_fp == -1)
    {
        perror("serialOpen():");
        return -1;
    }

    // Initialize WiringPi
    if (wiringPiSetup() == -1)
    {
        fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
        return -1;
    }

    // First send a thow-away command and ignore any response
    // just to make sure the BNO is in a good state and ready to accept
    // commands (this seems to be necessary after a hard power down).
    write_byte(BNO055_PAGE_ID_ADDR, (uint8_t)0x00, false);

    if (bno_set_mode(OPERATION_MODE_CONFIG) == -1)
    {
        fprintf(stderr, "Error changing op mode to config mode\n");
        close(serial_fp);
        return -1;
    }

    if (write_byte(BNO055_PAGE_ID_ADDR, (uint8_t)0x00, true) == -1)
    {
        fprintf(stderr, "Error initializing BNO055\n");
        close(serial_fp);
        return -1;
    }

    // Check the device ID
    uint8_t bnoId = read_byte(BNO055_CHIP_ID_ADDR);
    if (bnoId != BNO055_ID)
    {
        // Wait and try again
        sleep(1);
        bnoId = read_byte(BNO055_CHIP_ID_ADDR);
        if (bnoId != BNO055_ID)
        {
            fprintf(stderr, "Error: BNO055 Device ID does not match\nExpected: 0x%02x, Actual: 0x%02x\n", BNO055_ID, bnoId);
            close(serial_fp);
            return -1;
        }
    }

    // Issue a software reset command
    if (write_byte(BNO055_SYS_TRIGGER_ADDR, (uint8_t)0x20, false) == -1)
    {
        fprintf(stderr, "Error triggering software reset\n");
        close(serial_fp);
        return -1;
    }

    // Wait 650ms after reset
    struct timespec ts;
    ts.tv_sec = 650 / 1000;
    ts.tv_nsec = (650 % 1000) * 1000000;
    nanosleep(&ts, NULL);

    if (write_byte(BNO055_PWR_MODE_ADDR, (uint8_t)POWER_MODE_NORMAL, true) == -1)
    {
        fprintf(stderr, "Error entering normal power mode\n");
        close(serial_fp);
        return -1;
    }

    if (write_byte(BNO055_SYS_TRIGGER_ADDR, (uint8_t)0x00, true) == -1)
    {
        fprintf(stderr, "Error defaulting internal oscillator\n");
        close(serial_fp);
        return -1;
    }

    if (bno_set_mode(mode) == -1)
    {
        fprintf(stderr, "Error returning to operation mode\n");
        close(serial_fp);
        return -1;
    }

    return 1;
}
