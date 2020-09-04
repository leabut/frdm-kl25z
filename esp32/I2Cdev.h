/*
 * I2Cdev.h
 *
 *  Created on: 04.09.2020
 *      Author: spk
 */

#ifndef I2CDEV_H
#define I2CDEV_H

#include "driver/i2c.h"

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
esp_err_t i2c_read(uint8_t deviceAddress, uint8_t* data, size_t size);

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
esp_err_t i2c_write(uint8_t deviceAddress, uint8_t* data, size_t size);

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_init(void);

namespace I2Cdev {

bool readBit(uint8_t deviceAddress, uint8_t registerAddresss, uint8_t bitPosition, uint8_t* data);
bool writeBit(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition, uint8_t flag);
bool readByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data);
bool writeByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data);
bool readBits(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition, uint8_t length, uint8_t* data);
bool writeBits(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition, uint8_t length, uint8_t data);
bool readBytes(uint8_t deviceAddress, uint8_t registerAddress, uint8_t length, uint8_t* data);
bool writeWord(uint8_t deviceAddress, uint8_t registerAddress, uint16_t data);

}  // namespace I2Cdev

#endif /* I2CDEV_H */
