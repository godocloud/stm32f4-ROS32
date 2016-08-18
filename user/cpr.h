#ifndef __CPR_H__
#define __CPR_H__
#include "rtdef.h"

#define CPR_THREAD_PRIORITY   5

//#define USE_CPR_DUMMYDATA

/*  BH1772 Command Set  */
#define ALS_CONTROL           0x40     // ALS operation mode control and SW reset;
#define PS_CONTROL            0x41     // PS operation mode control;
#define I_LED                 0x42    //  LED current setting;
#define ALS_PS_MEAS           0x44    //Forced mode trigger;
#define PS_MEAS_RATE          0x45    //PS measurement rate;
#define ALS_MEAS_RATE         0x46    //ALS measurement rate;
#define ALS_DATA_0            0x4c    //ALS data (low Byte);
#define ALS_DATA_1            0x4d    //ALS data (high Byte);
#define ALS_PS_STATUS         0x4e    //measurement data and interrupt status;
#define PS_DATA               0x4f    //PS data;
#define BH1772_INTERRUPT      0x52   //interrupt setting;
#define PS_TH_H               0x53   //PS interrupt H threshold;
#define ALS_TH_UP_0           0x56   //ALS upper threshold low byte;
#define ALS_TH_UP_1           0x57   //ALS upper threshold high byte;
#define ALS_TH_LOW_0          0x58   //ALS lower threshold low byte;
#define ALS_TH_LOW_1          0x59   //ALS lower threshold high byte;
#define ALS_SENSITIVITY       0x5A   //ALS sensitivity   setting;
#define PERSISTENCE           0x5B    //INT pin INTERRUPT persistence setting;
#define PS_TH_L               0x5C    //PS interrupt L threshold;

/* ALS_CONTROL 0x40 */
#define  ALS Resolution       (1<<3)
#define  SW Reset             (1<<2)
#define  ALS_Standby_mode       (0)
#define  ALS_Forced_mode        (2)
#define  ALS_Stand_alone_mode   (3)

/*PS_CONTROL 0x41 */
#define  PS_Standby_mode      (0)
#define  PS_Forced_mode       (2)
#define  PS_Stand_alone_mode  (3)

/*I_LED 0x42*/
#define I_LED_5MA        (0)
#define I_LED_10MA       (1)
#define I_LED_20MA       (2)
#define I_LED_50MA       (3)
#define I_LED_100MA      (4)
#define I_LED_150MA      (5)
#define I_LED_200MA      (6)

/* ALS_PS_MEAS  0x44 */
#define ALS_trigger   (1<<1)
#define PS_trigger    (1<<0)

/*PS_MEAS_RATE 0x45 */
#define PS_meas_rate_10ms  (0)
#define PS_meas_rate_20ms  (1)
#define PS_meas_rate_30ms  (2)
#define PS_meas_rate_50ms  (3)
#define PS_meas_rate_70ms  (4)
#define PS_meas_rate_100ms (5)
#define PS_meas_rate_200ms (6)
#define PS_meas_rate_500ms (7)
#define PS_meas_rate_1000ms  (8)
#define PS_meas_rate_2000ms  (9)

/* ALS_MEAS_RATE 0x46 */
#define ALS_rate_disable    (1<<7)
#define ALS_meas_rate_100ms  (0)
#define ALS_meas_rate_200ms  (1)
#define ALS_meas_rate_500ms  (2)
#define ALS_meas_rate_1000ms  (3)
#define ALS_meas_rate_2000ms  (4)

/*ALS_PS_STATUS 0x4e */
#define ALS_INT_Status        (1<<7)
#define ALS_Data_Status       (1<<6)
#define PS_INT_Status         (1<<1)
#define PS_Data_Status        (1<<0)

/* PS_DATA     0x4F  */

/* BH1772 INTERRUPT 0x52 */
#define interrupt_source          (1<<5)
#define PS_interrupt_hysteresis   (1<<4)
#define Output_mode                (1<<3)
#define Interrupt_Polarity         (1<<2)
#define INT_pin_is_inactive         (0)
#define Triggered_by_only_PS_measurement  (1)
#define Triggered_by_only_ALS_measurement (2)
#define Triggered_by_PS_ALS_measurement   (3)

#define BH1772_I2C_Addr   0x29

struct IIC_device
{
    /* inherit from rt_device */
    struct rt_device parent;
    /* i2c mode */
    struct rt_i2c_bus_device * i2c_device;
};

/*BH1772电源控制IO口*/
#define BH1772_VCC_PORT      GPIOD
#define BH1772_VCC_CLK       RCC_APB2Periph_GPIOD
#define BH1772_VCC_PIN       GPIO_Pin_4
#define BH1772_VCC_ON        { GPIO_ResetBits(BH1772_VCC_PORT,BH1772_VCC_PIN);}
#define BH1772_VCC_OFF       { GPIO_SetBits(BH1772_VCC_PORT,BH1772_VCC_PIN);}


static rt_uint8_t CprDeviceSend(rt_uint16_t s_reg,rt_uint8_t s_data);
static rt_err_t CprDeviceInit(rt_device_t dev);
static rt_err_t CprDeviceOpen(rt_device_t dev, rt_uint16_t oflag);
static rt_err_t CprDeviceClose(rt_device_t dev);
static rt_err_t CprDeviceControl(rt_device_t dev, rt_uint8_t cmd, void *args);
static rt_uint8_t CprDeviceRec(rt_uint16_t s_reg,rt_uint8_t*buf);
rt_err_t CprDeviceHWInit(const char * i2c_bus_device_name);
static void CprHandleEntry(void* parameter);
void CprHandleInit(void);
static void SendDummyCprData(void);


#endif