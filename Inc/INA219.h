#ifndef __INA219_H
#define __INA219_H

#include "main.h"
#include "i2c.h"
//#include "tim.h"
// I2C Address Options
/*
#define INA219_I2C_ADDRESS_CONF_0               (uint8_t)(0x40 << 1)     // A0 = GND, A1 = GND
#define INA219_I2C_ADDRESS_CONF_1               (uint8_t)(0x41 << 1)     // A0 = VS+, A1 = GND
#define INA219_I2C_ADDRESS_CONF_2               (uint8_t)(0x42 << 1)     // A0 = SDA, A1 = GND
#define INA219_I2C_ADDRESS_CONF_3               (uint8_t)(0x43 << 1)     // A0 = SCL, A1 = GND
#define INA219_I2C_ADDRESS_CONF_4               (uint8_t)(0x44 << 1)     // A0 = GND, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_5               (uint8_t)(0x45 << 1)     // A0 = VS+, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_6               (uint8_t)(0x46 << 1)     // A0 = SDA, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_7               (uint8_t)(0x47 << 1)     // A0 = SCL, A1 = VS+
#define INA219_I2C_ADDRESS_CONF_8               (uint8_t)(0x48 << 1)     // A0 = GND, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_9               (uint8_t)(0x49 << 1)     // A0 = VS+, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_A               (uint8_t)(0x4A << 1)     // A0 = SDA, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_B               (uint8_t)(0x4B << 1)     // A0 = SCL, A1 = SDA
#define INA219_I2C_ADDRESS_CONF_C               (uint8_t)(0x4C << 1)     // A0 = GND, A1 = SCL
#define INA219_I2C_ADDRESS_CONF_D               (uint8_t)(0x4D << 1)     // A0 = VS+, A1 = SCL
#define INA219_I2C_ADDRESS_CONF_E               (uint8_t)(0x4E << 1)     // A0 = SDA, A1 = SCL
#define INA219_I2C_ADDRESS_CONF_F               (uint8_t)(0x4F << 1)     // A0 = SCL, A1 = SCL
*/
#define INA219_ADDRESS  (0x40 << 1)     // A0 = GND, A1 = GND

// 寄存器地址
#define INA219_REG_CONFIG                       (uint8_t)(0x00)      // 模式配置寄存器 (R/W)
#define INA219_REG_SHUNTVOLTAGE                 (uint8_t)(0x01)      // 分流电阻电压寄存器 (R)
#define INA219_REG_BUSVOLTAGE                   (uint8_t)(0x02)      // 总线电压寄存器 (R)
#define INA219_REG_POWER                        (uint8_t)(0x03)      // 功率寄存器 (R)
#define INA219_REG_CURRENT                      (uint8_t)(0x04)      // 电流寄存器 (R)
#define INA219_REG_CALIBRATION                  (uint8_t)(0x05)      // 基准值寄存器 (R/W)

//寄存器（0x01）参数定义
#define INA219_CONFIG_RESET                    (0x8000)  // Reset Bit

#define INA219_CONFIG_BVOLTAGERANGE_MASK       (0x2000)  // Bus Voltage Range Mask
#define INA219_CONFIG_BVOLTAGERANGE_16V        (0x0000)  // 0-16V Range
#define INA219_CONFIG_BVOLTAGERANGE_32V        (0x2000)  // 0-32V Range

#define INA219_CONFIG_GAIN_MASK                (0x1800)  // Gain Mask
#define INA219_CONFIG_GAIN_1_40MV              (0x0000)  // Gain 1, 40mV Range		//配置检测电阻上的最大检测电压
#define INA219_CONFIG_GAIN_2_80MV              (0x0800)  // Gain 2, 80mV Range
#define INA219_CONFIG_GAIN_4_160MV             (0x1000)  // Gain 4, 160mV Range
#define INA219_CONFIG_GAIN_8_320MV             (0x1800)  // Gain 8, 320mV Range

#define INA219_CONFIG_BADCRES_MASK             (0x0780)  // Bus ADC Resolution Mask
#define INA219_CONFIG_BADCRES_9BIT             (0x0080)  // 9-bit bus res = 0..511
#define INA219_CONFIG_BADCRES_10BIT            (0x0100)  // 10-bit bus res = 0..1023
#define INA219_CONFIG_BADCRES_11BIT            (0x0200)  // 11-bit bus res = 0..2047
#define INA219_CONFIG_BADCRES_12BIT            (0x0400)  // 12-bit bus res = 0..4097

#define INA219_CONFIG_SADCRES_MASK             (0x0078)  // Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_SADCRES_9BIT_1S_84US     (0x0000)  // 1 x 9-bit shunt sample
#define INA219_CONFIG_SADCRES_10BIT_1S_148US   (0x0008)  // 1 x 10-bit shunt sample
#define INA219_CONFIG_SADCRES_11BIT_1S_276US   (0x0010)  // 1 x 11-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_1S_532US   (0x0018)  // 1 x 12-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_2S_1060US  (0x0048)	 // 2 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_4S_2130US  (0x0050)  // 4 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_8S_4260US  (0x0058)  // 8 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_16S_8510US (0x0060)  // 16 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_32S_17MS   (0x0068)  // 32 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_64S_34MS   (0x0070)  // 64 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_128S_69MS  (0x0078)  // 128 x 12-bit shunt samples averaged together

#define INA219_CONFIG_MODE_MASK                (0x0007)  // Operating Mode Mask
#define INA219_CONFIG_MODE_POWERDOWN           (0x0000)
#define INA219_CONFIG_MODE_SVOLT_TRIGGERED     (0x0001)
#define INA219_CONFIG_MODE_BVOLT_TRIGGERED     (0x0002)
#define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED (0x0003)
#define INA219_CONFIG_MODE_ADCOFF              (0x0004)
#define INA219_CONFIG_MODE_SVOLT_CONTINUOUS    (0x0005)
#define INA219_CONFIG_MODE_BVOLT_CONTINUOUS    (0x0006)
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS (0x0007)	
//INA配置的参数
#define INA_R 0.01									//阻值大小（单位ohm）
#define INA_I_MAX	32								//硬件最大检测电流（单位：A）	INA219_CONFIG_GAIN_电压/INA_R
#define IAN_I_LSB	0.87							//(0x04)检测出值对应每位电流值	选择接近 = (INA_I_MAX/32767(2^15))*1000(为了得到单位为A)		(A)
#define INA_Power_LSB 20						//(0x03)检测出值对应每位功率值	= 20 * IAN_I_LSB	(MW)
#define INA_CAL 4096							//(0x05)寄存器基准值
//根据上面定义设置 配置寄存器（0x00）的值：0x3C1F
#define INA219_CONFIG_value	INA219_CONFIG_BVOLTAGERANGE_32V|INA219_CONFIG_GAIN_8_320MV|INA219_CONFIG_BADCRES_12BIT|INA219_CONFIG_SADCRES_12BIT_1S_532US|INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS

typedef struct
{
	uint16_t Voltage;
	uint16_t Current;
	uint16_t Power;
}INA219_t;

extern INA219_t INA219_D;

extern void INA_Init(void);
extern uint8_t INA_Write_Byte(uint8_t reg,uint16_t data); 				 
extern void INA_Read_Byte(uint8_t reg,uint8_t *R_Data);
extern void INA_GET_Voltage_MV(void);
extern void INA_GET_Current_MA(void);
extern void INA_GET_Power_MW(void);
#endif
