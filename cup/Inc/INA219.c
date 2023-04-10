#include "ina219.h"
int cnt20;
int cnt21;

INA219_t INA219_D;
void for_delay_us(uint32_t nus)
{
    uint32_t Delay = nus * 168/4;
    do
    {
        __NOP();
    }
    while (Delay --);
}

void INA_Init(void)
{
	INA_Write_Byte(INA219_REG_CONFIG,INA219_CONFIG_value);
	INA_Write_Byte(INA219_REG_CALIBRATION,INA_CAL);
};

//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
uint8_t INA_Write_Byte(uint8_t reg,uint16_t data) 				 
{ 
  uint8_t W_Data[2]={0};

  W_Data[0] = data>>8;
  W_Data[1] = data & 0xFF;

  HAL_I2C_Mem_Write_IT(&hi2c1, INA219_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, W_Data, 2);//��ȷ��16λIIC�Ƿ����
  HAL_Delay(100);
  
  return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������

void INA_Read_Byte(uint8_t reg,uint8_t *R_Data)
{
  HAL_I2C_Mem_Read_IT(&hi2c1, INA219_ADDRESS+0x01, reg, I2C_MEMADD_SIZE_8BIT, R_Data, 2);
	for_delay_us(100);
//  HAL_Delay(1);	
}
void INA_GET_Voltage_MV(void)	//��ȡ��ѹ����λ��mv��
{
	uint8_t data_temp[2];
	INA_Read_Byte(0x02,data_temp);
	INA219_D.Voltage=((((data_temp[0]<<8)+data_temp[1]) >> 3)*4);
}
void INA_GET_Current_MA(void)		//��ȡ��������λ��mA��
{
	uint8_t data_temp[2];
	INA_Write_Byte(INA219_REG_CONFIG,INA219_CONFIG_value);
	INA_Read_Byte(INA219_REG_CURRENT,data_temp);
	INA219_D.Current=((((data_temp[0]<<8)+data_temp[1]))*IAN_I_LSB);		//�õ��Ĵ�����ֵ�ڳ���ÿλ��Ӧ��ֵ��IAN_I_LSB���õ�ʵ�ʵĵ���
}
void INA_GET_Power_MW(void)		//��ȡ��ǰ���ʣ���λ��mw��
{
	uint8_t data_temp[2];
	INA_Read_Byte(INA219_REG_POWER,data_temp);
	INA219_D.Power=(((data_temp[0]<<8)+data_temp[1])*INA_Power_LSB);	//�õ��Ĵ�����ֵ�ڳ���ÿλ��Ӧ��ֵ��INA_Power_LSB���õ�ʵ�ʵĹ���
}

