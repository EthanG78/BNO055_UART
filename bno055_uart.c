/*
 * bno055_uart.c
 * BNO055_UART C Driver
 * Author: Ethan Garnier
 */
#include "bno055_uart.h"

int serial_fp = -1;

bno055_opmode_t op_mode = OPERATION_MODE_NDOF;

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
#ifdef DEBUG
        fprintf(stdout, "Serial Send: 0x");
        for (int i = 0; i < nCmdBytes; i++)
        {
            fprintf(stdout, "%02x", cmd[i]);
        }
        fprintf(stdout, "\n");
#endif

        // Flush the serial port
        serialFlush(serial_fp);

        // Send our command over the serial port
        if (write(serial_fp, cmd, nCmdBytes) == -1)
        {
            perror("send_serial_cmd() send error:");
            return -1;
        }

        delay(3);

        // If we don't want an ack, then return
        if (!ack)
            return 1;

        // This is independent on read or write command,
        // we want to read the first two response bytes
        // and determine if there was a bus error or not.
        int respByteIdx = 0;
        while (serialDataAvail(serial_fp) && respByteIdx < 2)
        {
            if (read(serial_fp, &resp[respByteIdx], 1) == -1)
            {
                perror("send_serial_cmd() timeout waiting for ack:");
                return -1;
            }
            respByteIdx++;
        }

        // TODO: WE NEED TO HAVE THIS WAIT FOR DATA
        // Otherwise, look for an acknowledgment
        /*if (read(serial_fp, resp, 2) == -1)
        {
            perror("send_serial_cmd() timeout waiting for ack:");
            return -1;
        }*/

#ifdef DEBUG
        fprintf(stdout, "Serial Receive: 0x%02x%02x\n", resp[0], resp[1]);
#endif

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
    if (resp == NULL || (ack && (resp[0] != 0xEE || resp[1] != 0x01)))
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
    {
        // wait and try again
        delay(1000);
        if (write_bytes(addr, bytes, 0x01, ack) == -1)
        {
            return -1;
        }
    }

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

    // Send read command over serial, only allow for
    // 5 attempts (ignoring bus errors)
    uint8_t resp[2];
    if (send_serial_cmd(cmd, 4, resp, true) == -1)
    {
        fprintf(stderr, "Error sending serial command\n");
        return -1;
    }

    // Process the response we received
    if (resp[0] != 0xBB || resp[1] != nBytes)
    {
        fprintf(stderr, "Failed to read register: 0x%02x%02x\n", resp[0], resp[1]);
        return -1;
    }

    // TODO: This is not ideal... we are calling read() for EVERY BTYE.
    int byteIdx = 0;
    while (serialDataAvail(serial_fp) && byteIdx < nBytes)
    {
        if (read(serial_fp, &bytes[byteIdx], 1) == -1)
        {
            perror("read_bytes() unable to read bytes:");
            return -1;
        }
    }

    // Read the bytes we requested
    /*if (read(serial_fp, bytes, (int)nBytes) == -1)
    {
        perror("read_bytes() unable to read bytes:");
        return -1;
    }*/

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
        // Wait and try again
        delay(1000);
        if (read_bytes(addr, readByte, 0x01) == -1)
        {
            fprintf(stderr, "Error reading single byte\n");
        }
    }

#ifdef DEBUG
    fprintf(stdout, "Byte read: 0x%02x\n", readByte[0]);
#endif

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

    delay(30);

    if (success == -1)
    {
        fprintf(stderr, "Error changing operating mode of BNO055\n");
        return -1;
    }

    return 1;
}

// Get the revision information of the BNO055 chip. These values
// are returned as a 6 btye array with the following format:
//      rev[0] = Software revision least significant byte
//      rev[1] = Software revision most significant byte
//      rev[2] = Bootloader version
//      rev[3] = Accelerometer ID
//      rev[4] = Magnetometer ID
//      rev[5] = Gyro ID
//
// Return 1 on success, -1 on error.
int bno_get_revision(uint8_t *rev)
{
    rev[0] = read_byte(BNO055_SW_REV_ID_LSB_ADDR);
    rev[1] = read_byte(BNO055_SW_REV_ID_MSB_ADDR);
    rev[2] = read_byte(BNO055_BL_REV_ID_ADDR);
    rev[3] = read_byte(BNO055_ACCEL_REV_ID_ADDR);
    rev[4] = read_byte(BNO055_MAG_REV_ID_ADDR);
    rev[5] = read_byte(BNO055_GYRO_REV_ID_ADDR);

    return 1;
}

// Check the status registers of the BNO055 chip. Return these values
// in a 3 byte array with the following format:
//      status[0] = System status register with following values
//          0x00 - Idle
//          0x01 - System Error
//          0x02 - Initializing Peripherals
//          0x03 - System Initialization
//          0x04 - Executing Self-Test
//          0x05 - Sensor fusion algorithm running
//          0x06 - System running without fusion algorithms
//      status[1] = System error register with following values
//          0x00 - No error
//          0x01 - Peripheral initialization error
//          0x02 - System initialization error
//          0x03 - Self test result failed
//          0x04 - Register map value out of range
//          0x05 - Register map address out of range
//          0x06 - Register map write error
//          0x07 - BNO low power mode not available for selected operation mode
//          0x08 - Accelerometer power mode not available
//          0x09 - Fusion algorithm configuration error
//          0x0A - Sensor configuration error
//      status[2] = Self test result register with following meaning
//          Bit value: 1 = test passed, 0 = test failed
//          Bit 0 = Accelerometer self test
//          Bit 1 = Magnetometer self test
//          Bit 2 = Gyroscope self test
//          Bit 3 = MCU self test
//          Value of 0x0F = all passed
//
// If run_self_test is passed in as false then no self test is performed and
// 0xF0 will be returned for the self test result.  Note that running a
// self test requires going into config mode which will stop the fusion
// engine from running.
//
// Return 1 on success, -1 on error.
int bno_get_system_status(uint8_t *status, bool run_self_test)
{
    // Read status and error registers
    status[0] = read_byte(BNO055_SYS_STAT_ADDR);
    status[1] = read_byte(BNO055_SYS_ERR_ADDR);

    // Self test result
    status[2] = 0xF0;
    if (run_self_test)
    {
        // Enter configuration mode
        if (bno_set_mode(OPERATION_MODE_CONFIG) == -1)
        {
            fprintf(stderr, "Error changing op mode to config mode\n");
            close(serial_fp);
            return -1;
        }

        // Perform self test
        uint8_t trigger = read_byte(BNO055_SYS_TRIGGER_ADDR);
        if (write_byte(BNO055_SYS_TRIGGER_ADDR, trigger | 0x10, true) == -1)
        {
            fprintf(stderr, "Failed to perform self test\n");
            close(serial_fp);
            return -1;
        }

        delay(1000);

        // Read the test results
        status[2] = read_byte(BNO055_SELFTEST_RESULT_ADDR);

        // Return to normal operation mode
        if (bno_set_mode(op_mode) == -1)
        {
            fprintf(stderr, "Error changing op mode to %02x mode\n", op_mode);
            close(serial_fp);
            return -1;
        }
    }

    return 1;
}

// Check the calibration status register of each sensor and return
// their values in a 4 byte array with the following format:
//      cal[0] = System calibration status (0x03 = calibrated, 0x00 = not calibrated)
//      cal[1] = Gyroscope calibration status (0x03 = calibrated, 0x00 = not calibrated)
//      cal[2] = Accelerometer calibration status (0x03 = calibrated, 0x00 = not calibrated)
//      cal[3] = Magnetometer calibration status (0x03 = calibrated, 0x00 = not calibrated)
//
// Return 1 on success, -1 on error.
int bno_get_calibration_status(uint8_t *cal)
{
    uint8_t calStatus = read_byte(BNO055_CALIB_STAT_ADDR);

    cal[0] = (calStatus >> 6) & 0x03;
    cal[1] = (calStatus >> 4) & 0x03;
    cal[2] = (calStatus >> 2) & 0x03;
    cal[3] = calStatus & 0x03;

    return 1;
}

// Fetch the sensor's calibration data and store this data in a
// bno055_offsets_t struct passed as an argument. This data may be 
// used to restore calibration using the bno_set_calibration()
// function.
//
// Return 1 on success, -1 on error.
int bno_get_calibration_data(bno055_offsets_t *offsets)
{
    // Enter configuration mode
    if (bno_set_mode(OPERATION_MODE_CONFIG) == -1)
    {
        fprintf(stderr, "Error changing op mode to config mode\n");
        close(serial_fp);
        return -1;
    }

    // Read the calibration data into a 22 byte array since the offset
    // data is stored at 22 contiguous bytes in memory
    uint8_t calData[22];
    if (read_bytes(serial_fp, calData, 22) == -1)
    {
        fprintf(stderr, "Unable to read calibration offset data\n");
        close(serial_fp);
        return -1;
    }

    // Must properly transfer data into bno055_offsets_t struct passed
    // TODO:

    return 1;
}

// Restore the sensor's calibration status using calibration offsets
// and radii stored within a bno055_offsets_t struct returned from
// bno_get_calibration_data() 
//
// Return 1 on success, -1 on error.
int bno_set_calibration(bno055_offsets_t offsets)
{
    // TODO:
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
    write_byte(BNO055_PAGE_ID_ADDR, 0x00, false);

    // Check the device ID
    uint8_t bnoId = read_byte(BNO055_CHIP_ID_ADDR);
    if (bnoId != BNO055_ID)
    {
        fprintf(stderr, "Error: BNO055 Device ID does not match\nExpected: 0x%02x, Actual: 0x%02x\n", BNO055_ID, bnoId);
        close(serial_fp);
        return -1;
    }

    // Store what mode the user wants to operate in
    op_mode = mode;

    // Enter configuration mode
    if (bno_set_mode(OPERATION_MODE_CONFIG) == -1)
    {
        fprintf(stderr, "Error changing op mode to config mode\n");
        close(serial_fp);
        return -1;
    }

    // Make sure we are on address page 0
    if (write_byte(BNO055_PAGE_ID_ADDR, 0x00, true) == -1)
    {
        fprintf(stderr, "Error initializing BNO055\n");
        close(serial_fp);
        return -1;
    }

    // Issue a software reset command
    if (write_byte(BNO055_SYS_TRIGGER_ADDR, 0x20, false) == -1)
    {
        fprintf(stderr, "Error triggering software reset\n");
        close(serial_fp);
        return -1;
    }

    delay(650);

    // Enter normal power mode
    if (write_byte(BNO055_PWR_MODE_ADDR, (uint8_t)POWER_MODE_NORMAL, true) == -1)
    {
        fprintf(stderr, "Error entering normal power mode\n");
        close(serial_fp);
        return -1;
    }

    // Default to internal oscillator
    if (write_byte(BNO055_SYS_TRIGGER_ADDR, 0x00, true) == -1)
    {
        fprintf(stderr, "Error defaulting internal oscillator\n");
        close(serial_fp);
        return -1;
    }

    // Return to normal operating mode as specified by mode argument
    if (bno_set_mode(op_mode) == -1)
    {
        fprintf(stderr, "Error returning to operation mode\n");
        close(serial_fp);
        return -1;
    }

    return 1;
}

// Close communication with BNO055. No other library
// functions may be called after this function finishes.
//
// Return 1 on success, -1 on error.
int bno_close()
{
    close(serial_fp);
    return 1;
}