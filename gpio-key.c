#include PLATFORM_HEADER
#include CONFIGURATION_HEADER

#include "stack/include/ember-types.h"
#include "event_control/event.h"

#include "hal/hal.h"
#include "hal/micro/micro.h"
#include EMBER_AF_API_GENERIC_INTERRUPT_CONTROL
#include "gpio-key.h"
//#include EMBER_AF_API_GPIO_SENSOR
#include "app/framework/include/af.h"

//设备不工作EM4 mode
#define GPIO_KEY_EM4WUPIN 0

#define KEY_ASSERT_DEBOUNCE   EMBER_AF_CUSTOM_GPIO_KEY_ASSERT_DEBOUNCE
#define KEY_DEASSERT_DEBOUNCE \
  EMBER_AF_CUSTOM_GPIO_KEY_DEASSERT_DEBOUNCE
#define KEY_IS_ACTIVE_HI EMBER_AF_CUSTOM_GPIO_KEY_POLARITY

//key interrupt event
EmberEventControl emberAfCustomGpioKeyInterruptEventControl;     //中断触发事件
EmberEventControl emberAfCustomGpioKeyDebounceEventControl;      //消抖事件

// State variables to track the status of the gpio key
static HalGpioKeyState lastKeyStatus = HAL_GPIO_KEY_ACTIVE;
static HalGpioKeyState newKeyStatus = HAL_GPIO_KEY_NOT_ACTIVE;

// structure used to store irq configuration from custom
static HalGenericInterruptControlIrqCfg *irqConfig;

static void keyDeassertedCallback(void);
static void keyAssertedCallback(void);
static void keyStateChangeDebounce(HalGpioKeyState status);



void halGpioKeyInitialize(void)
{
  uint8_t reedValue;

  // Set up the generic interrupt controller to handle changes on the gpio
  // key

  irqConfig = halGenericInterruptControlIrqCfgInitialize(GPIO_KEY_PIN,
                                                         GPIO_KEY_PORT,
                                                         GPIO_KEY_EM4WUPIN);

  halGenericInterruptControlIrqEventRegister(irqConfig,
                                             &emberAfCustomGpioKeyInterruptEventControl);
  halGenericInterruptControlIrqEnable(irqConfig);

  // Determine the initial value of the key.
  reedValue = halGenericInterruptControlIrqReadGpio(irqConfig);

  if (KEY_IS_ACTIVE_HI) {                   //1:有效
    if (reedValue) {
      newKeyStatus = HAL_GPIO_KEY_ACTIVE;
      lastKeyStatus = newKeyStatus;
    } else {
      newKeyStatus = HAL_GPIO_KEY_NOT_ACTIVE;
      lastKeyStatus = newKeyStatus;
    }
  } else {
    if (reedValue) {
      newKeyStatus = HAL_GPIO_KEY_NOT_ACTIVE;
      lastKeyStatus = newKeyStatus;
    } else {
      newKeyStatus = HAL_GPIO_KEY_ACTIVE;
      lastKeyStatus = newKeyStatus;
    }
  }


}


void emberAfCustomGpioKeyInterruptEventHandler(void)
{
  uint8_t reedValue;
  emberEventControlSetInactive(emberAfCustomGpioKeyInterruptEventControl);
  reedValue = halGenericInterruptControlIrqReadGpio(irqConfig);

  // If the gpio key was set to active high by the plugin properties, call
  // deassert when the value is 0 and assert when the value is 1.
  if (KEY_IS_ACTIVE_HI) {
    if (reedValue == 0) {
      keyDeassertedCallback();
    } else {
      keyAssertedCallback();
    }
  } else {
    if (reedValue == 0) {
        keyAssertedCallback();
    } else {
        keyDeassertedCallback();
    }
  }

}

// ------------------------------------------------------------------------------
// Custom private functions

// Helper function used to define action taken when a not yet debounced change
// in the gpio key is detected as having opened the switch.
static void keyDeassertedCallback(void)
{
  keyStateChangeDebounce(HAL_GPIO_KEY_NOT_ACTIVE);
}
// Helper function used to define action taken when a not yet debounced change
// in the gpio key is detected as having closed the switch
static void keyAssertedCallback(void)
{
  keyStateChangeDebounce(HAL_GPIO_KEY_ACTIVE);
}


// State machine used to debounce the gpio sensor.  This function is called on
// every transition of the gpio sensor's GPIO pin.  A delayed event is used to
// take action on the pin transition.  If the pin changes back to its original
// state before the delayed event can execute, that change is marked as a bounce
// and no further action is taken.
static void keyStateChangeDebounce(HalGpioKeyState status)
{
  if (status == lastKeyStatus) {
    // we went back to last status before debounce.  don't send the
    // message.
    emberEventControlSetInactive(emberAfCustomGpioKeyDebounceEventControl);
    return;
  }
  if (status == HAL_GPIO_KEY_ACTIVE) {
    newKeyStatus = status;
    emberEventControlSetDelayMS(emberAfCustomGpioKeyDebounceEventControl,
                                KEY_ASSERT_DEBOUNCE);
    return;
  } else if (status == HAL_GPIO_KEY_NOT_ACTIVE) {
    newKeyStatus = status;
    emberEventControlSetDelayMS(emberAfCustomGpioKeyDebounceEventControl,
                                KEY_DEASSERT_DEBOUNCE);
    return;
  }
}

void emberAfCustomGpioKeyDebounceEventHandler(void)
{
  emberEventControlSetInactive(emberAfCustomGpioKeyDebounceEventControl);
  lastKeyStatus = newKeyStatus;
  emberAfCustomGpioKeyStateChangedCallback(newKeyStatus);


}










