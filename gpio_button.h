#ifndef __GPIO_BUTTON_H__
#define __GPIO_BUTTON_H__

// User options for plugin GPIO Sensor Interface
#define EMBER_AF_CUSTOM_GPIO_BUTTON_POLARITY 0
#define EMBER_AF_CUSTOM_GPIO_BUTTON_ASSERT_DEBOUNCE 100
#define EMBER_AF_CUSTOM_GPIO_BUTTON_DEASSERT_DEBOUNCE 100
typedef enum {
  HAL_GPIO_BUTTON_ACTIVE = 0x01,
  HAL_GPIO_BUTTON_NOT_ACTIVE = 0x00,
} HalGpioButtonState;


void emberAfCustomGpioButtonStateChangedCallback(uint8_t);
void halGpioButtonInitialize(void);

#endif // __GPIO_BUTTON_H__
