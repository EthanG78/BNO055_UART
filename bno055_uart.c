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

// Get the revision information of the BNO055 chip. These
// values are returned through a bno055_rev_info_t struct passed
// as argument.
// Return 1 on success, -1 on error.
int bno_get_revision(bno055_rev_info_t *rev)
{
    memset(rev, 0, sizeof(bno055_rev_info_t));

    rev->accel_rev = read_byte(BNO055_ACCEL_REV_ID_ADDR);
    rev->gyro_rev = read_byte(BNO055_GYRO_REV_ID_ADDR);
    rev->mag_rev = read_byte(BNO055_MAG_REV_ID_ADDR);
    rev->bl_rev = read_byte(BNO055_BL_REV_ID_ADDR);

    uint8_t swLsb = read_byte(BNO055_SW_REV_ID_LSB_ADDR);
    uint8_t swMsb = read_byte(BNO055_SW_REV_ID_MSB_ADDR);

    rev->sw_rev = (((uint16_t)swMsb << 8) | swLsb) & 0xFFFF;

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
    if (read_bytes(ACCEL_OFFSET_X_LSB_ADDR, calData, 22) == -1)
    {
        fprintf(stderr, "Unable to read calibration offset data\n");
        close(serial_fp);
        return -1;
    }

    // TESTING

    // Must properly transfer data into bno055_offsets_t struct passed
    uint8_t accel_offset_x = (((uint16_t)calData[1] << 8) | calData[0]) & 0xFFFF;
    uint8_t accel_offset_y = (((uint16_t)calData[3] << 8) | calData[2]) & 0xFFFF;
    uint8_t accel_offset_z = (((uint16_t)calData[5] << 8) | calData[4]) & 0xFFFF;
    uint8_t mag_offset_x = (((uint16_t)calData[7] << 8) | calData[6]) & 0xFFFF;
    uint8_t mag_offset_y = (((uint16_t)calData[9] << 8) | calData[8]) & 0xFFFF;
    uint8_t mag_offset_z = (((uint16_t)calData[11] << 8) | calData[10]) & 0xFFFF;
    uint8_t gyro_offset_x = (((uint16_t)calData[13] << 8) | calData[12]) & 0xFFFF;
    uint8_t gyro_offset_y = (((uint16_t)calData[15] << 8) | calData[14]) & 0xFFFF;
    uint8_t gyro_offset_z = (((uint16_t)calData[17] << 8) | calData[16]) & 0xFFFF;
    uint8_t accel_radius = (((uint16_t)calData[19] << 8) | calData[18]) & 0xFFFF;
    uint8_t mag_radius = (((uint16_t)calData[21] << 8) | calData[20]) & 0xFFFF;

    fprintf(stdout, "TESTING bno_get_calibration_data()\n");
    fprintf(stdout, "Reading into byte array:\n%d ", accel_offset_x);
    fprintf(stdout, "\n%d ", accel_offset_y);
    fprintf(stdout, "\n%d ", accel_offset_z);
    fprintf(stdout, "\n%d ", mag_offset_x);
    fprintf(stdout, "\n%d ", mag_offset_y);
    fprintf(stdout, "\n%d ", mag_offset_z);
    fprintf(stdout, "\n%d ", gyro_offset_x);
    fprintf(stdout, "\n%d ", gyro_offset_y);
    fprintf(stdout, "\n%d ", gyro_offset_z);
    fprintf(stdout, "\n%d ", accel_radius);
    fprintf(stdout, "\n%d\n", mag_radius);

    offsets = (bno055_offsets_t *)&calData;

    fprintf(stdout, "Reading into bno055_offsets_t struct:\n%d ", offsets->accel_offset_x);
    fprintf(stdout, "\n%d ", offsets->accel_offset_y);
    fprintf(stdout, "\n%d ", offsets->accel_offset_z);
    fprintf(stdout, "\n%d ", offsets->mag_offset_x);
    fprintf(stdout, "\n%d ", offsets->mag_offset_y);
    fprintf(stdout, "\n%d ", offsets->mag_offset_z);
    fprintf(stdout, "\n%d ", offsets->gyro_offset_x);
    fprintf(stdout, "\n%d ", offsets->gyro_offset_y);
    fprintf(stdout, "\n%d ", offsets->gyro_offset_z);
    fprintf(stdout, "\n%d ", offsets->accel_radius);
    fprintf(stdout, "\n%d\n", offsets->mag_radius);

    // Return to normal operation mode
    if (bno_set_mode(op_mode) == -1)
    {
        fprintf(stderr, "Error changing op mode to %02x mode\n", op_mode);
        close(serial_fp);
        return -1;
    }

    return 1;
}

// Restore the sensor's calibration status using calibration offsets
// and radii stored within a bno055_offsets_t struct returned from
// bno_get_calibration_data()
//
// Return 1 on success, -1 on error.
int bno_set_calibration(bno055_offsets_t *offsets)
{
    // Enter configuration mode
    if (bno_set_mode(OPERATION_MODE_CONFIG) == -1)
    {
        fprintf(stderr, "Error changing op mode to config mode\n");
        close(serial_fp);
        return -1;
    }

    // Store calibration offsets in byte array
    uint8_t *calData = (uint8_t *)offsets;

    // Write the stored calibration offsets to their respective
    // registers on the bno055
    if (write_bytes(ACCEL_OFFSET_X_LSB_ADDR, calData, sizeof(bno055_offsets_t), true) == -1)
    {
        fprintf(stderr, "Unable to write calibration offset data\n");
        close(serial_fp);
        return -1;
    }

    // Return to normal operation mode
    if (bno_set_mode(op_mode) == -1)
    {
        fprintf(stderr, "Error changing op mode to %02x mode\n", op_mode);
        close(serial_fp);
        return -1;
    }

    return 1;
}

// Check if BNO055 sensors are fully calibrated
// based on the current operating mode. This code is adapted
// from Adafruit's Adafruit_BNO055 Arduino driver.
//
// Return 1 if fully calibrated, -1 otherwise
int bno_fully_calibrated()
{
    uint8_t cal[4];
    if (bno_get_calibration_status(&cal) == -1)
    {
        fprintf(stderr, "Error changing op mode to %02x mode\n", op_mode);
        return -1;
    }

    switch (op_mode)
    {
    case OPERATION_MODE_ACCONLY:
        return (cal[2] == 3);
    case OPERATION_MODE_MAGONLY:
        return (cal[3] == 3);
    case OPERATION_MODE_GYRONLY:
    case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
        return (cal[1] == 3);
    case OPERATION_MODE_ACCMAG:
    case OPERATION_MODE_COMPASS:
        return (cal[2] == 3 && cal[3] == 3);
    case OPERATION_MODE_ACCGYRO:
    case OPERATION_MODE_IMUPLUS:
        return (cal[2] == 3 && cal[1] == 3);
    case OPERATION_MODE_MAGGYRO:
        return (cal[3] == 3 && cal[1] == 3);
    default:
        return (cal[0] == 3 && cal[1] == 3 && cal[2] == 3 && cal[3] == 3);
    }
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