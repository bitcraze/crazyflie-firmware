#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "deckctrl_gpio.h"
#include "deckctrl.h"
#include "i2cdev.h" // @NO_MODULE
// @MODULE "../api/deckctrl_gpio.c"

I2cDrv deckBus;

static const DeckDiscoveryBackend_t deckctrl_backend = { .name = "deckctrl" };
static DeckCtrlContext deckctrl_context = { .i2cAddress = 0x44 };
static DeckInfo deck_info;
static uint8_t read_value;
static uint16_t expected_register;
static uint16_t actual_write_length;
static uint8_t actual_write_value;

bool i2cdevReadReg16(I2C_Dev *dev, uint8_t dev_address,
                     uint16_t mem_address, uint16_t length, uint8_t *data) {
  TEST_ASSERT_EQUAL_PTR(I2C1_DEV, dev);
  TEST_ASSERT_EQUAL_HEX8(deckctrl_context.i2cAddress, dev_address);
  TEST_ASSERT_EQUAL_HEX16(expected_register, mem_address);
  TEST_ASSERT_EQUAL_UINT16(1, length);
  *data = read_value;
  return true;
}

bool i2cdevWriteReg16(I2C_Dev *dev, uint8_t dev_address,
                      uint16_t mem_address, uint16_t length,
                      const uint8_t *data) {
  TEST_ASSERT_EQUAL_PTR(I2C1_DEV, dev);
  TEST_ASSERT_EQUAL_HEX8(deckctrl_context.i2cAddress, dev_address);
  TEST_ASSERT_EQUAL_HEX16(expected_register, mem_address);
  actual_write_length = length;
  actual_write_value = data[0];
  return true;
}

void setUp(void) {
  memset(&deck_info, 0, sizeof(deck_info));
  deck_info.discoveryBackend = &deckctrl_backend;
  deck_info.backendContext = &deckctrl_context;
  actual_write_length = 0;
  actual_write_value = 0;
}

void tearDown(void) {}

void testPin10DirectionUsesUpperBankAndPreservesOtherBits(void) {
  expected_register = DECKCTRL_GPIO_DIRECTION_REG + 1;
  read_value = 0xA1;

  TEST_ASSERT_TRUE(deckctrl_gpio_set_direction(&deck_info,
                                                DECKCTRL_GPIO_PIN_10,
                                                OUTPUT));
  TEST_ASSERT_EQUAL_UINT16(1, actual_write_length);
  TEST_ASSERT_EQUAL_HEX8(0xA5, actual_write_value);
}

void testPin11ValueUsesUpperBankAndPreservesOtherBits(void) {
  expected_register = DECKCTRL_GPIO_VALUE_REG + 1;
  read_value = 0xAF;

  TEST_ASSERT_TRUE(deckctrl_gpio_write(&deck_info, DECKCTRL_GPIO_PIN_11, LOW));
  TEST_ASSERT_EQUAL_UINT16(1, actual_write_length);
  TEST_ASSERT_EQUAL_HEX8(0xA7, actual_write_value);
}

void testPin3ReadUsesLowerBank(void) {
  expected_register = DECKCTRL_GPIO_VALUE_REG;
  read_value = 0x08;
  uint32_t value = LOW;

  TEST_ASSERT_TRUE(deckctrl_gpio_read(&deck_info, DECKCTRL_GPIO_PIN_3, &value));
  TEST_ASSERT_EQUAL_UINT32(HIGH, value);
}

void testPin11ReadUsesUpperBank(void) {
  expected_register = DECKCTRL_GPIO_VALUE_REG + 1;
  read_value = 0x80;
  uint32_t value = HIGH;

  TEST_ASSERT_TRUE(deckctrl_gpio_read(&deck_info, DECKCTRL_GPIO_PIN_11, &value));
  TEST_ASSERT_EQUAL_UINT32(LOW, value);
}
