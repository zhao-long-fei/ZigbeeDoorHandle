#ifndef __GPIO_KEY_H__
#define __GPIO_KEY_H__

// User options for plugin GPIO Sensor Interface
#define EMBER_AF_CUSTOM_GPIO_KEY_POLARITY 1
#define EMBER_AF_CUSTOM_GPIO_KEY_ASSERT_DEBOUNCE 100
#define EMBER_AF_CUSTOM_GPIO_KEY_DEASSERT_DEBOUNCE 100
typedef enum {
  HAL_GPIO_KEY_ACTIVE = 0x01,
  HAL_GPIO_KEY_NOT_ACTIVE = 0x00,
} HalGpioKeyState;


void emberAfCustomGpioKeyStateChangedCallback(uint8_t);
void halGpioKeyInitialize(void);

#endif // __GPIO_KEY_H__
