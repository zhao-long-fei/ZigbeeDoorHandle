/***************************************************************************//**
 * @file
 * @brief Callback implementation for ZigbeeMinimal sample application.
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

// This callback file is created for your convenience. You may add application
// code to this file. If you regenerate this file over a previous version, the
// previous version will be overwritten and any code you have added will be
// lost.

#include "app/framework/include/af.h"
#include "gpiointerrupt.h"
#include "em_gpio.h"
#include "em_iadc.h"
#include "gpio_button.h"
#include "gpio-key.h"
#include "app/framework/plugin/ias-zone-server/ias-zone-server.h"

#define GPIO_INTERRUPT_PORT gpioPortB                                           //门磁传感
#define GPIO_INTERRUPT_PIN  (1U)

#define GPIO_INTERRUPT_BUTTON_PORT gpioPortB                                    //门磁按键
#define GPIO_INTERRUPT_BUTTON_PIN  (0U)
#define DOOR_OPEN            1                                    //door 有效1为低电平
#define DOOR_CLOSE           0
#define TAMPER               1                                    //拆除
#define INSTALL              0                                    //安装
#define BUTTON_PRESS                1                             //按键按压
#define BUTTON_RELEASE              0                             //按键松开

#define PRESS_MIN_TIME              5000                          //按键入网  按下最小时间
#define PRESS_MAX_TIME              10000                         //按键入网  按下最多时间

//入网配网相关定义。
#define SECONDS_BETWEEN_JOIN_ATTEMPTS 15
#define QS_BETWEEN_JOIN_ATTEMPTS      (SECONDS_BETWEEN_JOIN_ATTEMPTS * 4)
#define REJOIN_ATTEMPTS  3 //重复入网次数
static uint8_t networkJoinAttempts = 0;

//连接网络和相关灯闪烁事件
EmberEventControl KeepConnectEventControl;                        //网络连接事件
EmberEventControl LedJoinNetworkStatusEventControl;               //入网状态灯闪烁事件
EmberEventControl CloseIndicateLedEventControl;

EmberEventControl SentZoneStatusEventControl;                     //状态，报警事件
EmberEventControl IADCCollectEventControl;                        //idc采集电压事件

// Set CLK_ADC to 10kHz (this corresponds to a sample rate of 1ksps)
#define CLK_SRC_IADC_FREQ       1000000 // CLK_SRC_IADC
#define CLK_ADC_FREQ            10000   // CLK_ADC
// When changing GPIO port/pins above, make sure to change xBUSALLOC macro's
// accordingly.
#define IADC_INPUT_BUS          CDBUSALLOC
#define IADC_INPUT_BUSALLOC     GPIO_CDBUSALLOC_CDEVEN0_ADC0
// Stores latest ADC sample and converts to volts
static volatile IADC_Result_t sample;
static volatile double singleResult;

void IadcInitialize(void);
uint16_t ZoneStatus;


uint32_t start_time;
uint32_t end_time;

#define EUI64_SIZE 8
#define BINDTABLE_SIZE 10
typedef uint8_t EmberEUI64[EUI64_SIZE];

uint16_t bindflag = 0;
uint8_t bindnumber = 0;
EmberBindingTableEntry BindTable;
EmberEUI64 returnEui64;
uint8_t SeekBuildBindTable(EmberStatus status);
/** @brief Stack Status
 *
 * This function is called by the application framework from the stack status
 * handler.  This callbacks provides applications an opportunity to be notified
 * of changes to the stack status and take appropriate action.  The return code
 * from this callback is ignored by the framework.  The framework will always
 * process the stack status after the callback returns.
 *
 * @param status   Ver.: always
 */
bool emberAfStackStatusCallback(EmberStatus status)
{
  // This value is ignored by the framework.
  uint8_t endpoint;
  uint8_t i;
  if(status == EMBER_NETWORK_DOWN)
  {
      halSetLed(0); //状态灯
      //做一下清除表的操作
      //emberClearBindingTable();
      //emberAfClearReportTableCallback();

      for (i = 0; i < emberAfEndpointCount(); i++) {
        endpoint = emberAfEndpointFromIndex(i);

        // now, clear the scene table for the current endpoint.
        emberAfScenesClusterClearSceneTableCallback(endpoint);
      }
  } else if(status == EMBER_NETWORK_UP){ //入网成功
      //关闭灯闪烁
      emberEventControlSetInactive(KeepConnectEventControl);
      emberEventControlSetInactive(LedJoinNetworkStatusEventControl);
      halClearLed(0);
      emberAfCorePrintln(" network status %x ", status);
      endpoint = 0x01;
      EmberNodeId id = 0x0000;
      EmberStatus status = emberAfPushEndpointNetworkIndex(endpoint);
      BindTable.networkIndex = emberGetCurrentNetwork();
      BindTable.local = endpoint;
      BindTable.remote = 0x01;
      BindTable.type = EMBER_UNICAST_BINDING;

      emberLookupEui64ByNodeId(id,returnEui64);
      /**eg:print:3A 2B F6 FE FF 27 71 84
       *    true :84 71 27 FF FE F6 2B 3A
       * */
      emberAfCorePrintln("%x %x %x %x %x %x %x %x  ",  returnEui64[0],
                                                       returnEui64[1],
                                                       returnEui64[2],
                                                       returnEui64[3],
                                                       returnEui64[4],
                                                       returnEui64[5],
                                                       returnEui64[6],
                                                       returnEui64[7]);
      strcpy((char*)BindTable.identifier,(char*)returnEui64);
      emberAfCorePrintln("1 %x %x %x %x %x %x %x %x  ",  BindTable.identifier[0],
                                                         BindTable.identifier[1],
                                                         BindTable.identifier[2],
                                                         BindTable.identifier[3],
                                                         BindTable.identifier[4],
                                                         BindTable.identifier[5],
                                                         BindTable.identifier[6],
                                                         BindTable.identifier[7]);
      BindTable.clusterId = 0x0001;
      //status=emberSetBinding(6, &BindTable);
      SeekBuildBindTable(status);




  }else {     //其他情况同样关闭灯闪烁
      emberAfCorePrintln("else network status %x ", status);
  }
  return false;
}
uint8_t SeekBuildBindTable(EmberStatus status)
{

  //遍历绑定表
  uint8_t index;
  for(index=0;index<EMBER_BINDING_TABLE_SIZE;index++){
      EmberBindingTableEntry result;
      status = emberGetBinding(index,&result);
      emberAfCorePrintln("status %d",status);
      emberAfCorePrintln("result->clusterId= %2x",result.clusterId);
     if((result.clusterId == 0x0000)&&(result.local==0x01)){
         bindflag |= 0x0001;          //有0x0000的cluster的绑定表
     }else if(result.clusterId == 0x0001){
         bindflag |= 0x0002;          //有0x0001的cluster的绑定表
     }else if(result.clusterId == 0x0020){
         bindflag |= 0x0004;          //有0x0020的cluster的绑定表
     }else if(result.clusterId == 0x0500){
         bindflag |= 0x0008;          //有0x0500的cluster的绑定表
     }else {
         //此工程只需要四个绑定表
     }
     emberAfCorePrintln("bindflag %2x %x %x %x %x %x %x %x %x ",bindflag, result.identifier[0],
                                                                            result.identifier[1],
                                                                            result.identifier[2],
                                                                            result.identifier[3],
                                                                            result.identifier[4],
                                                                            result.identifier[5],
                                                                            result.identifier[6],
                                                                            result.identifier[7]);
     if((result.identifier[7] == returnEui64[7])&&(result.identifier[6] == returnEui64[6])){
         //存在绑定表
         bindnumber++;
     }
  }

  if(!(bindflag & 0x0001)){  //没有cluster0x0000的绑定表
      //建立绑定表     BindTable需要初始化
      BindTable.clusterId = 0x0000;
      status=emberSetBinding(bindnumber++, &BindTable);
      emberAfCorePrintln("B0");
  }else{
      EmberBindingTableEntry *Result;
      emberGetBinding(bindnumber, Result);
      emberAfCorePrintln("Result->clusterId= %2x "   , Result->clusterId);
      emberAfCorePrintln("Result->identifier= %x %x %x %x %x %x %x %x ", Result->identifier[0],
                                                                         Result->identifier[1],
                                                                         Result->identifier[2],
                                                                         Result->identifier[3],
                                                                         Result->identifier[4],
                                                                         Result->identifier[5],
                                                                         Result->identifier[6],
                                                                         Result->identifier[7]);
      emberAfCorePrintln("Result->networkIndex= %x ", Result->networkIndex);
      emberAfCorePrintln("Result->type = %x ", Result->type);
  }
  if(!(bindflag & 0x0002)){  //没有cluster0x0001的绑定表
      BindTable.clusterId = 0x0001;
      status=emberSetBinding(bindnumber++, &BindTable);
      emberAfCorePrintln("B1");
  }
  if(!(bindflag & 0x0004)){  //没有cluster0x0020的绑定表
      BindTable.clusterId = 0x0020;
      status=emberSetBinding(bindnumber++, &BindTable);
      emberAfCorePrintln("B2");
  }
  if(!(bindflag & 0x0008)){  //没有cluster0x0500的绑定表
      BindTable.clusterId = 0x0500;
      status=emberSetBinding(bindnumber++, &BindTable);
      emberAfCorePrintln("B3");
  }

  emberAfCorePrintln("bindflag %d bindnumber %d ",bindflag,bindnumber);
  (void) emberAfPopNetworkIndex();
  return status;
}




/** @brief Complete
 *
 * This callback is fired when the Network Steering plugin is complete.
 *
 * @param status On success this will be set to EMBER_SUCCESS to indicate a
 * network was joined successfully. On failure this will be the status code of
 * the last join or scan attempt. Ver.: always
 * @param totalBeacons The total number of 802.15.4 beacons that were heard,
 * including beacons from different devices with the same PAN ID. Ver.: always
 * @param joinAttempts The number of join attempts that were made to get onto
 * an open Zigbee network. Ver.: always
 * @param finalState The finishing state of the network steering process. From
 * this, one is able to tell on which channel mask and with which key the
 * process was complete. Ver.: always
 */
void emberAfPluginNetworkSteeringCompleteCallback(EmberStatus status,
                                                  uint8_t totalBeacons,
                                                  uint8_t joinAttempts,
                                                  uint8_t finalState)
{
  emberAfCorePrintln("%p network %p: 0x%X", "Join", "complete", status);
}

/** @brief Ok To Sleep
 *
 * This function is called by the Idle/Sleep plugin before sleeping. It is
 * called with interrupts disabled. The application should return true if the
 * device may sleep or false otherwise.
 *
 * @param durationMs The maximum duration in milliseconds that the device will
 * sleep. Ver.: always
 */
bool emberAfPluginIdleSleepOkToSleepCallback(uint32_t durationMs)
{
  /*关闭看门狗，去睡觉*/
  halInternalDisableWatchDog(MICRO_DISABLE_WATCH_DOG_KEY);


  //NVIC_EnableIRQ(USART1_RX_IRQn);
  return true;
}

/** @brief Wake Up
 *
 * This function is called by the Idle/Sleep plugin after sleeping.
 *
 * @param durationMs The duration in milliseconds that the device slept.
 * Ver.: always
 */
void emberAfPluginIdleSleepWakeUpCallback(uint32_t durationMs)
{
  /**/
  /*醒来，开启看门狗*/
  halInternalEnableWatchDog();

}

void emberAfMainInitCallback(void)
{
  halGpioButtonInitialize();
  //配网按钮
  GPIO_PinModeSet(GPIO_INTERRUPT_BUTTON_PORT, GPIO_INTERRUPT_BUTTON_PIN,gpioModeInputPull,1); //1:DOUT  1:默认高电平低电平触发  0：默认低电平 高电平触发
  halGpioKeyInitialize();
  //防拆按键
  GPIO_PinModeSet(GPIO_KEY_PORT, GPIO_KEY_PIN,gpioModeInputPull,0); //1:DOUT  1:默认高电平低电平触发  0：默认低电平 高电平触发
  //门磁
  GPIO_PinModeSet(GPIO_INTERRUPT_PORT, GPIO_INTERRUPT_PIN,gpioModeInputPull,1); //1:DOUT  1:默认高电平低电平触发   0：默认低电平 高电平触发
  GPIOINT_Init();//在此中断初始化 中断插件才能生效，，
  CMU_ClockEnable(cmuClock_GPIO, true);

  IadcInitialize();
  emberEventControlSetDelayMS(IADCCollectEventControl,0);//上电后立刻上报电量

  bindflag = 0;
  bindnumber = 0;
}

/** @brief Called whenever the GPIO sensor detects a change in state.
 *
 * @param newSensorState The new state of the sensor based alarm
 * (EMBER_AF_PLUGIN_GPIO_SENSOR_ACTIVE or
 * EMBER_AF_PLUGIN_GPIO_SENSOR_NOT_ACTIVE)  Ver.: always
 */
void emberAfPluginGpioSensorStateChangedCallback(uint8_t newSensorState)
{
  //门磁检测IO
  emberAfCorePrintln("GpioSensorStateChangedCallback %d",newSensorState);
  static uint8_t data[2] = {0,0};

  emberAfReadServerAttribute(1,
                             ZCL_IAS_ZONE_CLUSTER_ID,
                             ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                             (uint8_t*)data,
                             8);
  //低电平有效  有效为1   1:开门  0:关门
  if(DOOR_OPEN==newSensorState){
    data[0] |= 0x01;                                            //alarm
    emberAfWriteServerAttribute(1,
                                ZCL_IAS_ZONE_CLUSTER_ID,
                                ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                                (uint8_t*)data,
                                ZCL_BITMAP16_ATTRIBUTE_TYPE);
  }else if(DOOR_CLOSE==newSensorState){
    data[0] &= 0xFE;                                            //alarm
    emberAfWriteServerAttribute(1,
                                ZCL_IAS_ZONE_CLUSTER_ID,
                                ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                                (uint8_t*)data,
                                ZCL_BITMAP16_ATTRIBUTE_TYPE);
  }
  emberAfCorePrintln("data[0]=%d",data[0]);
  ZoneStatus = ((data[0]) & 0x00ff) +(data[1]<<8 & 0xff00);
  emberAfCorePrintln("GpioSensorStateZoneStatus=%d",ZoneStatus);
  //emberEventControlSetDelayMS(SentZoneStatusEventControl,0);
  emberEventControlSetActive(SentZoneStatusEventControl);

}
//入网
void emberAfCustomGpioButtonStateChangedCallback(uint8_t newButtonState)
{
  emberAfCorePrintln("GpioButtonStateChangedCallback %d",newButtonState);
  if(BUTTON_PRESS==newButtonState){                  //按键按下
      start_time = halCommonGetInt32uMillisecondTick();
      halSetLed(0);                                  //PA00     LED无关中断 可以设置其他的IO
      emberEventControlSetDelayMS(CloseIndicateLedEventControl,5000);
  }else if(BUTTON_RELEASE==newButtonState) {         //按键松开
      halClearLed(0);                                //关灯
      end_time = halCommonGetInt32uMillisecondTick();
      if(end_time>start_time){
          if((end_time-start_time)<PRESS_MIN_TIME){                        // 小于5s
                    emberAfCorePrintln("<5s");
                }else if ((end_time-start_time)>PRESS_MIN_TIME && (end_time-start_time)<PRESS_MAX_TIME){
                                                                                 // 5-10s 入网
                  emberAfCorePrintln("KeepConnectEventControl");
                  emberEventControlSetActive(KeepConnectEventControl);           // 开启入网事件
                  emberEventControlSetActive(LedJoinNetworkStatusEventControl);  // 开启入网灯闪烁事件 绿灯闪烁
                }else {                                                          // >10s 什么都不做
                    emberAfCorePrintln(">10s");
                }
      }
  }else { //计数溢出 从零开始了 基本不可能，49天溢出一次

  }

}


//防拆
void emberAfCustomGpioKeyStateChangedCallback(uint8_t newKeyStatus)
{
  emberAfCorePrintln("emberAfCustomGpioKeyStateChangedCallback %d",newKeyStatus);
  static uint8_t data[2] = {0,0};

  emberAfReadServerAttribute(1,
                             ZCL_IAS_ZONE_CLUSTER_ID,
                             ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                             (uint8_t*)data,
                             8);
  //高电平有效  有效为1   1:拆动 0:正常
  if(TAMPER==newKeyStatus){
    data[0] |= 0x04;                                            //tamper
    emberAfWriteServerAttribute(1,
                                ZCL_IAS_ZONE_CLUSTER_ID,
                                ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                                (uint8_t*)data,
                                ZCL_BITMAP16_ATTRIBUTE_TYPE);
  }else if(INSTALL==newKeyStatus){
    data[0] &= 0xFB;                                            //alarm
    emberAfWriteServerAttribute(1,
                                ZCL_IAS_ZONE_CLUSTER_ID,
                                ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                                (uint8_t*)data,
                                ZCL_BITMAP16_ATTRIBUTE_TYPE);
  }
  emberAfCorePrintln("data[0]=%d",data[0]);
  ZoneStatus = ((data[0]) & 0x00ff) +(data[1]<<8 & 0xff00);
  emberAfCorePrintln("GpioKeyStateChangedCallback=%d",ZoneStatus);
  emberEventControlSetActive(SentZoneStatusEventControl);

}

/**
 * 入网事件
 * */
void KeepConnectEventHandler(void)
{    //入网成功后要关灯
  emberEventControlSetInactive(KeepConnectEventControl);
  if (emberAfNetworkState() == EMBER_NO_NETWORK)  //不在网状态去入网
  {
      if (networkJoinAttempts < REJOIN_ATTEMPTS)
      {
          networkJoinAttempts++;
          emberAfPluginNetworkSteeringStart();
          emberEventControlSetDelayQS(KeepConnectEventControl,QS_BETWEEN_JOIN_ATTEMPTS);
      }else{
          emberEventControlSetInactive(KeepConnectEventControl);
          emberEventControlSetInactive(LedJoinNetworkStatusEventControl);
          halClearLed(0);
          networkJoinAttempts=0;

      }
  }else if (emberAfNetworkState() == EMBER_JOINED_NETWORK) {
      emberLeaveNetwork(); //离开网络

      networkJoinAttempts=0;
      emberClearBindingTable();
      bindflag = 0;
      bindnumber = 0;
      emberEventControlSetDelayMS(KeepConnectEventControl,1000);         //1s后继续入网
      emberEventControlSetDelayMS(LedJoinNetworkStatusEventControl,1000);

  }
}
/**
 * 入网灯闪烁事件
 * */
void LedJoinNetworkStatusEventHandler(void)
{
  emberEventControlSetInactive(LedJoinNetworkStatusEventControl);
  halToggleLed(0);
  emberEventControlSetDelayMS(LedJoinNetworkStatusEventControl,400);
}

void CloseIndicateLedEventHandler(void)
{
  //按键按下5秒后执行关灯事件
  emberEventControlSetInactive(CloseIndicateLedEventControl);
  halClearLed(0);
}

void SentZoneStatusEventHandler(void)
{
  emberEventControlSetInactive(SentZoneStatusEventControl);
  emberAfCorePrintln("ZoneStatus=%d",ZoneStatus);
  emberAfFillCommandIasZoneClusterZoneStatusChangeNotification( ZoneStatus,
                                                                0, // extended status, must be zero per spec
                                                                emberAfPluginIasZoneServerGetZoneId(1),
                                                                0); // called "delay" in the spec
  emberAfSetCommandEndpoints(1, 1);
  emberAfSendCommandUnicastToBindings();

}

void IadcInitialize(void)
{
  // Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

  // Reset IADC to reset configuration in case it has been modified
  IADC_reset(IADC0);
  // Configure IADC clock source for use while in EM2
  // Note that HFRCOEM23 is the lowest frequency source available for the IADC
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_HFRCOEM23);

  // Modify init structs and initialize
  init.warmup = iadcWarmupKeepWarm;
  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0,CLK_SRC_IADC_FREQ, 0);

  // Configuration 0 is used by both scan and single conversions by default
  // Use unbuffered AVDD as reference
  initAllConfigs.configs[0].reference = iadcCfgReferenceVddx;
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                     CLK_ADC_FREQ,
                                                                     0,
                                                                     iadcCfgModeNormal,
                                                                     init.srcClkPrescale);
  // Single initialization
  initSingle.dataValidLevel = _IADC_SCANFIFOCFG_DVL_VALID1;
  // Set conversions to run one,continuously
  initSingle.triggerAction = iadcTriggerActionOnce;
  // Configure Input sources for single ended conversion
  initSingleInput.posInput = iadcPosInputPortCPin4;
  initSingleInput.negInput = iadcNegInputGnd;

  // Initialize the ADC peripheral
  IADC_init(IADC0, &init, &initAllConfigs);
  // Initialize Scan
  IADC_initSingle(IADC0, &initSingle,&initSingleInput);

  // Allocate the analog bus for ADC0 inputs
  GPIO->IADC_INPUT_BUS |= IADC_INPUT_BUSALLOC;

  // Enable interrupts on data valid level
  IADC_enableInt(IADC0, IADC_IF_SINGLEFIFODVL);

  // Enable ADC interrupts
  NVIC_ClearPendingIRQ(IADC_IRQn);
  NVIC_EnableIRQ(IADC_IRQn);
}

/**************************************************************************//**
 * @brief  ADC Handler
 *****************************************************************************/
void IADC_IRQHandler(void)
{
  uint8_t voltage;

  // Read data from the FIFO
  sample = IADC_pullSingleFifoResult(IADC0);
  emberAfCorePrintln("sample.data=%d ",sample.data);
  // For single-ended the result range is 0 to +Vref, i.e., 12 bits for the
  // conversion value.
  singleResult = sample.data * 3.3 / 0xFFF;
  emberAfCorePrintln("singleResult=%d ",(int)(singleResult*10));
  //emberAfCorePrintln("singleResult1=%e ",singleResult);
  IADC_clearInt(IADC0, IADC_IF_SINGLEFIFODVL);
  voltage = sample.data*200/0xFFF;
  emberAfCorePrintln("voltage=%d ",voltage);
  emberAfWriteServerAttribute(1,
                             ZCL_POWER_CONFIG_CLUSTER_ID,
                             ZCL_BATTERY_PERCENTAGE_REMAINING_ATTRIBUTE_ID,     //此属性是否上报可自己设置
                             (uint8_t*)&voltage,
                             ZCL_DATA8_ATTRIBUTE_TYPE);
  //电量低于2.5的时候提醒电量低
  //200-3.3  0.1 约= 6  3.3-2.5 = 0.8  2.2-3.3 --> 152 --> 200
 static uint8_t data[2] = {0,0};
 emberAfReadServerAttribute(1,
                           ZCL_IAS_ZONE_CLUSTER_ID,
                           ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                           (uint8_t*)data,
                           8);
  if(voltage<153) {
      data[0] |= 0x08;                                            //alarm
      emberAfWriteServerAttribute(1,
                                  ZCL_IAS_ZONE_CLUSTER_ID,
                                  ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                                  (uint8_t*)data,
                                  ZCL_BITMAP16_ATTRIBUTE_TYPE);
  }else {
      data[0] &= 0xF7;                                            //取消 alarm
      emberAfWriteServerAttribute(1,
                                  ZCL_IAS_ZONE_CLUSTER_ID,
                                  ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                                  (uint8_t*)data,
                                  ZCL_BITMAP16_ATTRIBUTE_TYPE);
  }
}

void IADCCollectEventHandler(void)
{
  emberEventControlSetInactive(IADCCollectEventControl);
  IADC_command(IADC0, iadcCmdStartSingle);
  emberEventControlSetDelayMS(IADCCollectEventControl,10000); //先取消电量的周期性上报。
}
