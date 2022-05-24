#include PLATFORM_HEADER
#include CONFIGURATION_HEADER

#include "stack/include/ember-types.h"
#include "event_control/event.h"

#include "hal/hal.h"
#include "hal/micro/micro.h"
#include EMBER_AF_API_GENERIC_INTERRUPT_CONTROL
#include "gpio_button.h"
//#include EMBER_AF_API_GPIO_SENSOR
#include "app/framework/include/af.h"

//设备不工作EM4 mode
#define GPIO_BUTTON_EM4WUPIN 0

#define BUTTON_ASSERT_DEBOUNCE   EMBER_AF_CUSTOM_GPIO_BUTTON_ASSERT_DEBOUNCE
#define BUTTON_DEASSERT_DEBOUNCE \
  EMBER_AF_CUSTOM_GPIO_BUTTON_DEASSERT_DEBOUNCE
#define BUTTON_IS_ACTIVE_HI EMBER_AF_CUSTOM_GPIO_BUTTON_POLARITY

//button interrupt event
EmberEventControl emberAfCustomGpioButtonInterruptEventControl;     //中断触发事件
EmberEventControl emberAfCustomGpioButtonDebounceEventControl;      //消抖事件

// State variables to track the status of the gpio button
static HalGpioButtonState lastSensorStatus = HAL_GPIO_BUTTON_ACTIVE;
static HalGpioButtonState newSensorStatus = HAL_GPIO_BUTTON_NOT_ACTIVE;

// structure used to store irq configuration from custom
static HalGenericInterruptControlIrqCfg *irqConfig;

static void buttonDeassertedCallback(void);
static void buttonAssertedCallback(void);
static void buttonStateChangeDebounce(HalGpioButtonState status);

void halGpioButtonInitialize(void)
{
  uint8_t reedValue;

  // Set up the generic interrupt controller to handle changes on the gpio
  // button

  irqConfig = halGenericInterruptControlIrqCfgInitialize(GPIO_BUTTON_PIN,
                                                         GPIO_BUTTON_PORT,
                                                         GPIO_BUTTON_EM4WUPIN);

  halGenericInterruptControlIrqEventRegister(irqConfig,
                                             &emberAfCustomGpioButtonInterruptEventControl);
  halGenericInterruptControlIrqEnable(irqConfig);

  // Determine the initial value of the button.
  reedValue = halGenericInterruptControlIrqReadGpio(irqConfig);

  if (BUTTON_IS_ACTIVE_HI) {                   //1:有效
    if (reedValue) {
      newSensorStatus = HAL_GPIO_BUTTON_ACTIVE;
      lastSensorStatus = newSensorStatus;
    } else {
      newSensorStatus = HAL_GPIO_BUTTON_NOT_ACTIVE;
      lastSensorStatus = newSensorStatus;
    }
  } else {
    if (reedValue) {
      newSensorStatus = HAL_GPIO_BUTTON_NOT_ACTIVE;
      lastSensorStatus = newSensorStatus;
    } else {
      newSensorStatus = HAL_GPIO_BUTTON_ACTIVE;
      lastSensorStatus = newSensorStatus;
    }
  }


}


void emberAfCustomGpioButtonInterruptEventHandler(void)
{
  uint8_t reedValue;
  emberEventControlSetInactive(emberAfCustomGpioButtonInterruptEventControl);
  reedValue = halGenericInterruptControlIrqReadGpio(irqConfig);

  // If the gpio button was set to active high by the plugin properties, call
  // deassert when the value is 0 and assert when the value is 1.
  if (BUTTON_IS_ACTIVE_HI) {
    if (reedValue == 0) {
      buttonDeassertedCallback();
    } else {
      buttonAssertedCallback();
    }
  } else {
    if (reedValue == 0) {
      buttonAssertedCallback();
    } else {
      buttonDeassertedCallback();
    }
  }

}

void emberAfCustomGpioButtonDebounceEventHandler(void)
{
  emberEventControlSetInactive(emberAfCustomGpioButtonDebounceEventControl);
  lastSensorStatus = newSensorStatus;
  emberAfCustomGpioButtonStateChangedCallback(newSensorStatus);


}


// ------------------------------------------------------------------------------
// Plugin private functions

// Helper function used to define action taken when a not yet debounced change
// in the gpio button is detected as having opened the switch.
static void buttonDeassertedCallback(void)
{
  buttonStateChangeDebounce(HAL_GPIO_BUTTON_NOT_ACTIVE);
}
// Helper function used to define action taken when a not yet debounced change
// in the gpio button is detected as having closed the switch
static void buttonAssertedCallback(void)
{
  buttonStateChangeDebounce(HAL_GPIO_BUTTON_ACTIVE);
}


// State machine used to debounce the gpio sensor.  This function is called on
// every transition of the gpio sensor's GPIO pin.  A delayed event is used to
// take action on the pin transition.  If the pin changes back to its original
// state before the delayed event can execute, that change is marked as a bounce
// and no further action is taken.
static void buttonStateChangeDebounce(HalGpioButtonState status)
{
  if (status == lastSensorStatus) {
    // we went back to last status before debounce.  don't send the
    // message.
    emberEventControlSetInactive(emberAfCustomGpioButtonDebounceEventControl);
    return;
  }
  if (status == HAL_GPIO_BUTTON_ACTIVE) {
    newSensorStatus = status;
    emberEventControlSetDelayMS(emberAfCustomGpioButtonDebounceEventControl,
                                BUTTON_ASSERT_DEBOUNCE);
    return;
  } else if (status == HAL_GPIO_BUTTON_NOT_ACTIVE) {
    newSensorStatus = status;
    emberEventControlSetDelayMS(emberAfCustomGpioButtonDebounceEventControl,
                                BUTTON_DEASSERT_DEBOUNCE);
    return;
  }
}


