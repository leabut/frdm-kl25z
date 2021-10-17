/*
 * I2Cdev.cpp
 *
 *  Created on: 04.09.2020
 *      Author: spk
 */
#include "I2Cdev.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_SCL_IO 22         /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21         /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(0) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000    /*!< I2C master clock frequency */

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

esp_err_t i2c_read_robust(uint8_t deviceAddress, uint8_t* data, size_t size) {
  if (size == 0) {
    return ESP_OK;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, deviceAddress << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, data[0], ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int ret = i2c_master_cmd_begin(0 /* i2c_num */, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    return ret;
  }
  vTaskDelay(30 / portTICK_RATE_MS);
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, deviceAddress << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, static_cast<i2c_ack_type_t>(NACK_VAL));
  //i2c_master_read_byte(cmd, data_l, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(0 /* i2c_num */, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

esp_err_t i2c_read(uint8_t deviceAddress, uint8_t* data, size_t size) {
  if (size == 0) {
    return ESP_OK;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, deviceAddress << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, data[0], ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int ret = i2c_master_cmd_begin(0 /* i2c_num */, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    return ret;
  }
  vTaskDelay(1 / portTICK_RATE_MS);
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, deviceAddress << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, static_cast<i2c_ack_type_t>(NACK_VAL));
  //i2c_master_read_byte(cmd, data_l, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(0 /* i2c_num */, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

esp_err_t i2c_write(uint8_t deviceAddress, uint8_t* data, size_t size) {
  if (size == 0) {
    return ESP_OK;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (deviceAddress << 1) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write(cmd, data, size, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(0 /* i2c_num */, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

esp_err_t i2c_init(void) {
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf{};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

namespace I2Cdev {

bool readBit(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition, uint8_t* data) {
  *data = registerAddress;
  if (i2c_read(deviceAddress, data, 1u) != 0u) {
    return false;
  }

  *data &= 0x01u << bitPosition;
  return true;
}

bool writeBit(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition, uint8_t flag) {
  uint8_t request[2] = {registerAddress, 0u};
  if (i2c_read_robust(deviceAddress, request, 1u) != 0u) {
    return false;
  }

  if (flag > 1u) {
    flag = 1u;
  }

  uint8_t mask = 0u;
  if (flag == 1u) {
    mask |= (flag << bitPosition);
  } else {
    mask &= ~(flag << bitPosition);
  }

  request[1] = request[0] & mask;
  request[0] = registerAddress;
  if (i2c_write(deviceAddress, request, 2) != 0u) {
    return false;
  }

  return true;
}

bool readByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* data) {
  *data = registerAddress;
  if (i2c_read(deviceAddress, data, 1u) != 0u) {
    return false;
  }

  return true;
}

bool writeByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data) {
  uint8_t request[2] = {registerAddress, data};
  if (i2c_write(deviceAddress, request, 2) != 0u) {
    return false;
  }

  return true;
}

bool readBits(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition, uint8_t length, uint8_t* data) {
  *data = registerAddress;
  if (i2c_read(deviceAddress, data, 1u) != 0u) {
    return false;
  }

  uint8_t mask = ((1 << length) - 1) << (bitPosition - length + 1);
  *data &= mask;
  *data >>= (bitPosition - length + 1);
  return true;
}

bool writeBits(uint8_t deviceAddress, uint8_t registerAddress, uint8_t bitPosition, uint8_t length, uint8_t data) {
  uint8_t request[2] = {registerAddress, 0u};
  if (i2c_read_robust(deviceAddress, request, 1u) != 0u) {
    return false;
  }

  uint8_t mask = ((1 << length) - 1) << (bitPosition - length + 1);
  data <<= (bitPosition - length + 1);  // shift data into correct position
  data &= mask;                         // zero all non-important bits in data
  request[0] &= ~(mask);                // zero all important bits in existing byte
  request[0] |= data;                   // combine data with existing byte
  return writeByte(deviceAddress, registerAddress, request[0]);
}

bool readBytes(uint8_t deviceAddress, uint8_t registerAddress, uint8_t length, uint8_t* data) {
  *data = registerAddress;
  if (i2c_read(deviceAddress, data, length) != 0u) {
    return false;
  }

  return true;
}

bool writeWord(uint8_t deviceAddress, uint8_t registerAddress, uint16_t data) {
  uint8_t request[3] = {registerAddress, static_cast<uint8_t>(data >> 8u), static_cast<uint8_t>(data)};
  if (i2c_write(deviceAddress, request, 3) != 0u) {
    return false;
  }

  return true;
}

}  // namespace I2Cdev
