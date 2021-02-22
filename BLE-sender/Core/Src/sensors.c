/*
 * sensors.c
 *
 *  Created on: 29 gen 2021
 *      Author: UTPM9
 */


#include "sensors.h"
#include "stm32l4xx_hal.h"
#include <stdlib.h>

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;

int T_C0=0;
int T_C1=0;
int16_t T_C0_lsb=0;
int16_t T_C1_lsb=0;
float m=0;


uint8_t H_0=0;
uint8_t H_1=0;
int16_t H_0_lsb=0;
int16_t H_1_lsb=0;
float mh=0;


void initLPS22hh(){
	uint8_t addressWrite=0xba;
	uint8_t turnOn[]={0x10,0x20};//The address of the register and the value of the register to turn on the sensor

    HAL_I2C_Master_Transmit(&hi2c1,addressWrite,turnOn,2,1);
}

void getPressure(float *pressure){

	int lsb;
	uint8_t addressWrite=0xba;
	uint8_t addressRead=0xbb;
	uint8_t pressXL[]={0x28};
	uint8_t pressL[]={0x29};
	uint8_t pressH[]={0x2a};
	uint8_t data[2];

	HAL_I2C_Master_Transmit(&hi2c1,addressWrite,pressXL,1,1);
	HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
	lsb=data[0];

	HAL_I2C_Master_Transmit(&hi2c1,addressWrite,pressL,1,1);
	HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
	lsb|=data[0]<<8;

	HAL_I2C_Master_Transmit(&hi2c1,addressWrite,pressH,1,1);
	HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
	lsb|=data[0]<<16;


	if(lsb>8388607){
		lsb=lsb-1;
		lsb=~lsb;
	}

	*pressure=((float)lsb)/((float)(4096));

}

void initHTS221(){
	uint8_t addressWrite=0xbe;
	uint8_t addressRead=0xbf;
	uint8_t turnOn[]={0x20,0x81};//The address of the register and the value of the register to turn on the sensor
    uint8_t data[2];

	HAL_I2C_Master_Transmit(&hi2c1,addressWrite,turnOn,2,1);


    uint8_t tempMinAddress[]={0x32};
    uint8_t tempMaxAddress[]={0x33};
    //reading low temperature calibration lsb
    HAL_I2C_Master_Transmit(&hi2c1,addressWrite,tempMinAddress,1,1);
    HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
    T_C0=data[0];

    //reading high temperature calibration lsb
    HAL_I2C_Master_Transmit(&hi2c1,addressWrite,tempMaxAddress,1,1);
    HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
    T_C1=data[0];

	uint8_t MSB_temp[]={0x35};
	HAL_I2C_Master_Transmit(&hi2c1,addressWrite,MSB_temp,1,1);
	HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);

	T_C0|=((data[0]&0x03)<<8);
	T_C1|=(((data[0]&0x0c)>>2)<<8);

	T_C0=T_C0>>3;
	T_C1=T_C1>>3;

	uint8_t ADC0L[]={0x3c};
	uint8_t ADC0H[]={0x3d};
	//get the calibration adc min
	HAL_I2C_Master_Transmit(&hi2c1,addressWrite,ADC0L,1,1);
	HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
	T_C0_lsb=data[0];

	HAL_I2C_Master_Transmit(&hi2c1,addressWrite,ADC0H,1,1);
	HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
	T_C0_lsb|=data[0]<<8;


    uint8_t ADC1L[]={0x3e};
    uint8_t ADC1H[]={0x3f};
    //leggo temperatura
    HAL_I2C_Master_Transmit(&hi2c1,addressWrite,ADC1L,1,1);
    HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
    T_C1_lsb=data[0];

    HAL_I2C_Master_Transmit(&hi2c1,addressWrite,ADC1H,1,1);
    HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
    T_C1_lsb|=data[0]<<8;

    m=((float)(T_C1-T_C0))/((float)(T_C1_lsb-T_C0_lsb));


    uint8_t HumMinAddress[]={0x30};
     uint8_t HumMaxAddress[]={0x31};
     //reading low temperature calibration lsb
     HAL_I2C_Master_Transmit(&hi2c1,addressWrite,HumMinAddress,1,1);
     HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
     H_0=data[0];

     //reading high temperature calibration lsb
     HAL_I2C_Master_Transmit(&hi2c1,addressWrite,HumMaxAddress,1,1);
     HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
     H_1=data[0];


 	H_0=H_0>>1;
 	H_1=H_1>>1;

 	ADC0L[0]=0x36;
 	ADC0H[0]=0x37;
 	//get the calibration adc min
 	HAL_I2C_Master_Transmit(&hi2c1,addressWrite,ADC0L,1,1);
 	HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
 	H_0_lsb=data[0];

 	HAL_I2C_Master_Transmit(&hi2c1,addressWrite,ADC0H,1,1);
 	HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
 	H_0_lsb|=data[0]<<8;


    ADC1L[0]=0x3a;
    ADC1H[0]=0x3b;
     //leggo temperatura
     HAL_I2C_Master_Transmit(&hi2c1,addressWrite,ADC1L,1,1);
     HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
     H_1_lsb=data[0];

     HAL_I2C_Master_Transmit(&hi2c1,addressWrite,ADC1H,1,1);
     HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
     H_1_lsb|=data[0]<<8;

     mh=((float)(H_1-H_0))/((float)(H_1_lsb-H_0_lsb));



}

void getHumidity(float *humidity){

    uint8_t humL[]={0x28};
    uint8_t humH[]={0x29};
    uint8_t data[2];
    uint8_t addressWrite=0xbe;
	uint8_t addressRead=0xbf;
	int16_t hum;
    //reading temperature
    HAL_I2C_Master_Transmit(&hi2c1,addressWrite,humL,1,1);
    HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
    hum=data[0];

    //high register
    HAL_I2C_Master_Transmit(&hi2c1,addressWrite,humH,1,1);
    HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
    hum|=data[0]<<1;

    *humidity=H_0+mh*hum;

}



void getTemperature(float *temperature){

    uint8_t tempL[]={0x2A};
    uint8_t tempH[]={0x2B};
    uint8_t data[2];
    uint8_t addressWrite=0xbe;
	uint8_t addressRead=0xbf;
	int16_t temp;
    //reading temperature
    HAL_I2C_Master_Transmit(&hi2c1,addressWrite,tempL,1,1);
    HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
    temp=data[0];

    //high register
    HAL_I2C_Master_Transmit(&hi2c1,addressWrite,tempH,1,1);
    HAL_I2C_Master_Receive(&hi2c1,addressRead,data,1,1);
    temp|=data[0]<<8;

    *temperature=T_C0+m*temp;

}



int8_t getMicrophone(){
    //will become a function
	 int microphone;
     HAL_ADC_Start(&hadc1);
     while(HAL_ADC_PollForConversion(&hadc1, 1000000)!=HAL_OK);
     microphone = HAL_ADC_GetValue(&hadc1);
     int16_t valueDb;
	 valueDb=getMicrophonedb(microphone-2048);
     //updateValueChar(CUSTOM_SERVICE_HANDLE,MIC_CHAR_HANDLE,15,2,valueDb);

     return valueDb;
}



void init_accelerometer(){

	  //Turn on the accelerometer, it will be a function in the sensors library
	  uint8_t ACCcmd[2];
	  uint8_t ACCread[2];
	  ACCcmd[0]=0x20;
	  ACCcmd[1]=0x13;
	  HAL_GPIO_WritePin(CS_LIS2DW12_GPIO_Port,CS_LIS2DW12_Pin,GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1,ACCcmd,ACCread,2,1);
	  HAL_GPIO_WritePin(CS_LIS2DW12_GPIO_Port,CS_LIS2DW12_Pin,GPIO_PIN_SET);

}


void getAxisAccelerometer(int16_t *accx, int16_t *accy, int16_t *accz){

	  uint8_t ACCcmd[2];
	  uint8_t ACCread[2];


	   //ACC X

	   ACCcmd[0]=0xA8;
	   HAL_GPIO_WritePin(CS_LIS2DW12_GPIO_Port,CS_LIS2DW12_Pin,GPIO_PIN_RESET);
	   HAL_SPI_TransmitReceive(&hspi1,ACCcmd,ACCread,2,1);
	   HAL_GPIO_WritePin(CS_LIS2DW12_GPIO_Port,CS_LIS2DW12_Pin,GPIO_PIN_SET);
	   *accx=ACCread[1]>>2;

	   ACCcmd[0]=0xA9;
	   HAL_GPIO_WritePin(CS_LIS2DW12_GPIO_Port,CS_LIS2DW12_Pin,GPIO_PIN_RESET);
	   HAL_SPI_TransmitReceive(&hspi1,ACCcmd,ACCread,2,1);
	   HAL_GPIO_WritePin(CS_LIS2DW12_GPIO_Port,CS_LIS2DW12_Pin,GPIO_PIN_SET);
	   *accx|=((ACCread[1])<<6);

	   if(*accx>8191){
		   *accx|=0xC000;
		   *accx=*accx-1;
		   *accx=~*accx;
		   *accx=*accx>>2;
		   *accx=-*accx;
	   }else{
		   *accx=*accx>>2;
	   }
	   //ACC Y



	   ACCcmd[0]=0xAA;
	   HAL_GPIO_WritePin(CS_LIS2DW12_GPIO_Port,CS_LIS2DW12_Pin,GPIO_PIN_RESET);
	   HAL_SPI_TransmitReceive(&hspi1,ACCcmd,ACCread,2,1);
	   HAL_GPIO_WritePin(CS_LIS2DW12_GPIO_Port,CS_LIS2DW12_Pin,GPIO_PIN_SET);
	   *accy=ACCread[1]>>2;

	   ACCcmd[0]=0xAB;
	   HAL_GPIO_WritePin(CS_LIS2DW12_GPIO_Port,CS_LIS2DW12_Pin,GPIO_PIN_RESET);
	   HAL_SPI_TransmitReceive(&hspi1,ACCcmd,ACCread,2,1);
	   HAL_GPIO_WritePin(CS_LIS2DW12_GPIO_Port,CS_LIS2DW12_Pin,GPIO_PIN_SET);
	   *accy|=((ACCread[1])<<6);

	   if(*accy>8191){
		   *accy|=0xC000;
		   *accy=*accy-1;
		   *accy=~*accy;
		   *accy=*accy>>2;
		   *accy=-*accy;
	   }else{
		   *accy=*accy>>2;
	   }



	   //ACC Z

	   ACCcmd[0]=0xAC;
	   HAL_GPIO_WritePin(CS_LIS2DW12_GPIO_Port,CS_LIS2DW12_Pin,GPIO_PIN_RESET);
	   HAL_SPI_TransmitReceive(&hspi1,ACCcmd,ACCread,2,1);
	   HAL_GPIO_WritePin(CS_LIS2DW12_GPIO_Port,CS_LIS2DW12_Pin,GPIO_PIN_SET);
	   *accz=ACCread[1]>>2;

	   ACCcmd[0]=0xAD;
	   HAL_GPIO_WritePin(CS_LIS2DW12_GPIO_Port,CS_LIS2DW12_Pin,GPIO_PIN_RESET);
	   HAL_SPI_TransmitReceive(&hspi1,ACCcmd,ACCread,2,1);
	   HAL_GPIO_WritePin(CS_LIS2DW12_GPIO_Port,CS_LIS2DW12_Pin,GPIO_PIN_SET);
	   *accz|=((ACCread[1])<<6);

	   if(*accz>8191){
		   *accz|=0xC000;
		   *accz=*accz-1;
		   *accz=~*accz;
		   *accz=*accz>>2;
		   *accz=-*accz;
	   }else{
		   *accz=*accz>>2;
	   }



}

int16_t getMicrophonedb(int analogValue){

	if(analogValue<=0){
		analogValue=-analogValue;
	}

	uint8_t bit=getfirstValidBit(analogValue);

	return 47+(bit*3);


}

int getfirstValidBit(int absoluteValue){
	uint8_t counter;
	counter=0;

	while(absoluteValue>0){
	absoluteValue=absoluteValue>>1;
	counter++;
	}
	return counter;
}

void init_magnetometer(){

	  uint8_t MAGcmd[2];
	  MAGcmd[0]=0x00|0x60;
	  MAGcmd[1]=0x90;
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi3,MAGcmd,2,1);
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_SET);

	  MAGcmd[0]=0x00|0x61;
	  MAGcmd[1]=0x03;
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi3,MAGcmd,2,1);
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_SET);



	  MAGcmd[0]=0x00|0x62;
	  MAGcmd[1]=0x30;
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi3,MAGcmd,2,1);
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_SET);






}


void getAxisMagnetometer(int16_t *magx, int16_t *magy, int16_t *magz){

	  uint8_t MAGcmd[2];
	  uint8_t MAGread[2];



	  MAGcmd[0]=0x80|0x68;
	  MAGcmd[1]=0x00;
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi3,MAGcmd,1,1);
	  HAL_SPI_Receive(&hspi3,MAGread,1,1);
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_SET);
	  *magx=MAGread[0];

	  MAGcmd[0]=0x80|0x69;
	  MAGcmd[1]=0x00;
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi3,MAGcmd,1,1);
	  HAL_SPI_Receive(&hspi3,MAGread,1,1);
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_SET);
	  *magx|=MAGread[0]<<8;


	  MAGcmd[0]=0x80|0x6A;
	  MAGcmd[1]=0x00;
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi3,MAGcmd,1,1);
	  HAL_SPI_Receive(&hspi3,MAGread,1,1);
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_SET);
	  *magy=MAGread[0];

	  MAGcmd[0]=0x80|0x6B;
	  MAGcmd[1]=0x00;
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi3,MAGcmd,1,1);
	  HAL_SPI_Receive(&hspi3,MAGread,1,1);
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_SET);
	  *magy|=MAGread[0]<<8;


	  MAGcmd[0]=0x80|0x6C;
	  MAGcmd[1]=0x00;
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi3,MAGcmd,1,1);
	  HAL_SPI_Receive(&hspi3,MAGread,1,1);
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_SET);
	  *magz=MAGread[0];

	  MAGcmd[0]=0x80|0x6D;
	  MAGcmd[1]=0x00;
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi3,MAGcmd,1,1);
	  HAL_SPI_Receive(&hspi3,MAGread,1,1);
	  HAL_GPIO_WritePin(CS_LIS2MDL_GPIO_Port,CS_LIS2MDL_Pin,GPIO_PIN_SET);
	  *magz|=MAGread[0]<<8;



}

void init_gyroscope(){

		  uint8_t GYROcmd[2];
		  uint8_t GYROread[2];
		  GYROcmd[0]=0x00|0x11;
		  GYROcmd[1]=0x12;
		  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive(&hspi1,GYROcmd,GYROread,2,1);
		  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_SET);







}

void getAxisGyro(int16_t *gyrox, int16_t *gyroy, int16_t *gyroz){

	  uint8_t GYROcmd[2];
	  uint8_t GYROread[2];

	  GYROcmd[0]=0x80|0x22;
	  GYROcmd[1]=0x00;
	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1,GYROcmd,GYROread,2,1);
	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_SET);

	  *gyrox=GYROread[1];

	  GYROcmd[0]=0x80|0x23;
	  GYROcmd[1]=0x00;
	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1,GYROcmd,GYROread,2,1);
	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_SET);

	  *gyrox|=GYROread[1]<<8;

	  *gyrox=*gyrox*4;

	  GYROcmd[0]=0x80|0x24;
	  GYROcmd[1]=0x00;
	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1,GYROcmd,GYROread,2,1);
	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_SET);

	  *gyroy=GYROread[1];



	  GYROcmd[0]=0x80|0x25;
	  GYROcmd[1]=0x00;
	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1,GYROcmd,GYROread,2,1);
	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_SET);

	  *gyroy|=GYROread[1]<<8;

	  *gyroy=*gyroy*4;

	  GYROcmd[0]=0x80|0x26;
	  GYROcmd[1]=0x00;

	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1,GYROcmd,GYROread,2,1);
	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_SET);

	  *gyroz=GYROread[1];

	  GYROcmd[0]=0x80|0x27;
	  GYROcmd[1]=0x00;
	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1,GYROcmd,GYROread,2,1);
	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_SET);

	  *gyroz|=GYROread[1]<<8;

	  *gyroz=*gyroz*4;

	  *gyrox=*gyrox/1000;
	  *gyroy=*gyroy/1000;
	  *gyroz=*gyroz/1000;

}
