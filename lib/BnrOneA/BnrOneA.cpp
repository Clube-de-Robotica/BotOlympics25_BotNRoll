/*
  BnrOneA.cpp - Library for interfacing with Bot'n Roll ONE Arduino Compatible from www.botnroll.com
  Created by Jos� Cruz, November 28, 2013.
  Updated June 06, 2018. -> String type variable for printing on LCD
  Updated January 04, 2019. -> Improved SPI communication. New line read and calibrate routines.
  Released into the public domain.
*/
#include "SPI.h"
#include "BnrOneA.h"
#include "EEPROM.h"

int delayTR=20; //20 MinStable:15  Crash:14
int delaySS=20; //20 Crash: No crash even with 0 (ZERO)
///////////////////////////////////////////////////////////////////////
//setup routines
///////////////////////////////////////////////////////////////////////

void BnrOneA::spiConnect(byte sspin)
{
    _sspin=sspin;
    pinMode(_sspin,OUTPUT); //sspin as output
    SPI.begin();    //Initializes the SPI bus by setting SCK and MOSI to outputs, pulling SCK and MOSI low.
    SPI.setBitOrder(MSBFIRST); //Sets the order of the bits shifted out of and into the SPI bus MSBFIRST (most-significant bit first).
    SPI.setDataMode(SPI_MODE1); //Sets the SPI data mode: that is, clock polarity and phase.
	SPI.setClockDivider(SPI_CLOCK_DIV2);// 2 4 8 16 32 64 64 128
    digitalWrite(_sspin, HIGH); // SPI in hold state by pulling SS high.
    delayMicroseconds(delaySS);//Minimo � 8
	readFirmware(&fmw1,&fmw2,&fmw3);//
//	Serial.print(fmw1);Serial.print(".");Serial.print(fmw2);Serial.print(".");Serial.println(fmw3);
}
///////////////////////////////////////////////////////////////////////
//private routines
///////////////////////////////////////////////////////////////////////
byte BnrOneA::spiRequestByte(byte command)
{
    byte value=(byte)0xFF;
	int i;
    byte buffer[]={KEY1,KEY2};
	byte numBytes=2;
	digitalWrite(_sspin, LOW); // Select the SPI Slave device to start communication.
    byte dummy=SPI.transfer(command);        // Sends one byte
    delayMicroseconds(delayTR);
    for(i=0; i<numBytes;i++)
    {
        dummy=SPI.transfer(buffer[i]);  // Sends one byte
		delayMicroseconds(delayTR);
    }	
    value=SPI.transfer(0x00);  // Reads one byte    
    digitalWrite(_sspin, HIGH); // Close communication with slave device.	SPI.endTransaction();
	delayMicroseconds(delaySS); 
 return value;
}

int BnrOneA::spiRequestWord(byte command)
{
    byte value[2]={0,0};
    int i;
    byte buffer[]={KEY1,KEY2};
	byte numBytes=2;
	digitalWrite(_sspin, LOW); // Select the SPI Slave device to start communication.
    byte dummy=SPI.transfer(command);        // Sends one byte
    delayMicroseconds(delayTR);
    for(i=0; i<numBytes;i++)
    {
        dummy=SPI.transfer(buffer[i]);  // Sends one byte
		delayMicroseconds(delayTR);
    }
    for (i=0; i<2; i++)
    {
        value[i]=SPI.transfer(0x00);  // Reads one byte
        delayMicroseconds(delayTR);
    }
    i=0;
    i=value[0];
    i=i<<8;
    i=i+value[1];
    digitalWrite(_sspin, HIGH); // Close communication with slave device.
	delayMicroseconds(delaySS);
  return i;
}
void BnrOneA::spiSendData(byte command, byte buffer[], byte numBytes)
{
	byte dummy;
    digitalWrite(_sspin, LOW); // Select the SPI Slave device to start communication.
    dummy=SPI.transfer(command);        // Sends one byte
    delayMicroseconds(delayTR);
    for (int k =0; k< numBytes;k++)
    {
        dummy=SPI.transfer(buffer[k]);  // Sends one byte
		delayMicroseconds(delayTR);
    }
    digitalWrite(_sspin, HIGH); // Close communication with slave device.	
	delayMicroseconds(delaySS);
}
///////////////////////////////////////////////////////////////////////
//Write routines
///////////////////////////////////////////////////////////////////////
void BnrOneA::move(int speedL,int speedR)
{
    byte speedL_H=highByte(speedL);
    byte speedL_L=lowByte(speedL);
    byte speedR_H=highByte(speedR);
    byte speedR_L=lowByte(speedR);

    byte buffer[]={KEY1,KEY2,speedL_H,speedL_L,speedR_H,speedR_L};
    spiSendData(COMMAND_MOVE,buffer,sizeof(buffer));
    delay(2);//Wait while command is processed
}
void BnrOneA::moveCalibrate(int powerL,int powerR)
{
    byte powerL_H=highByte(powerL);
    byte powerL_L=lowByte(powerL);
    byte powerR_H=highByte(powerR);
    byte powerR_L=lowByte(powerR);

    byte buffer[]={KEY1,KEY2,powerL_H,powerL_L,powerR_H,powerR_L};
    spiSendData(COMMAND_MOVE_CALIBRATE,buffer,sizeof(buffer));
    delay(2);//Wait while command is processed
}
void BnrOneA::move1m(byte motor,int speed)
{
    byte speed_H=highByte(speed);
    byte speed_L=lowByte(speed);

    byte buffer[]={KEY1,KEY2,motor,speed_H,speed_L};
    spiSendData(COMMAND_MOVE_1M,buffer,sizeof(buffer));
    delay(2);//Wait while command is processed
}
void BnrOneA::stop()
{
    byte buffer[]={KEY1,KEY2};
    spiSendData(COMMAND_STOP,buffer,sizeof(buffer));
    delay(2);//Wait while command is processed
}
void BnrOneA::stop1m(byte motor)
{
    byte buffer[]={KEY1,KEY2,motor};
    spiSendData(COMMAND_STOP_1M,buffer,sizeof(buffer));
    delay(2);//Wait while command is processed
}
void BnrOneA::brake(byte torqueL,byte torqueR)
{
    byte buffer[]={KEY1,KEY2,torqueL,torqueR};
    spiSendData(COMMAND_BRAKE,buffer,sizeof(buffer));
    delay(2);//Wait while command is processed
}
void BnrOneA::brake1m(byte motor,byte torque)
{
    byte buffer[]={KEY1,KEY2,motor,torque};
    spiSendData(COMMAND_BRAKE_1M,buffer,sizeof(buffer));
    delay(2);//Wait while command is processed
}
void BnrOneA::brake1m(byte motor)
{
    byte buffer[]={KEY1,KEY2,motor,BRAKE_TORQUE};
    spiSendData(COMMAND_BRAKE_1M,buffer,sizeof(buffer));
    delay(2);//Wait while command is processed
}
void BnrOneA::resetEncL()
{
    byte buffer[]={KEY1,KEY2};
    spiSendData(COMMAND_ENCL_RESET,buffer,sizeof(buffer));
    delay(2);//Wait while command is processed
}
void BnrOneA::resetEncR()
{
    byte buffer[]={KEY1,KEY2};
    spiSendData(COMMAND_ENCR_RESET,buffer,sizeof(buffer));
    delay(2);//Wait while command is processed
}
void BnrOneA::led(boolean state)
{
    if(state>1)
    {
        state=1;
    }
    byte buffer[]={KEY1,KEY2,(byte)state};
    spiSendData(COMMAND_LED,buffer,sizeof(buffer));
    delay(2);//Wait while command is processed
}
void BnrOneA::obstacleEmitters(boolean state)
{
    if(state>1)
    {
        state=1;
    }
    byte buffer[]={KEY1,KEY2,(byte)state};
    spiSendData(COMMAND_IR_EMITTERS,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::servo1(byte position)
{
    // SAFETY LIMITS ADDED FOR BOTOLYMPICS
    position = constrain(position, 0, 90);
    byte buffer[]={KEY1,KEY2,position};
    spiSendData(COMMAND_SERVO1,buffer,sizeof(buffer));
    delay(2);//Wait while command is processed
}
void BnrOneA::servo2(byte position)
{
    // SAFETY LIMITS ADDED FOR BOTOLYMPICS
    position = constrain(position, 0, 90);
    byte buffer[]={KEY1,KEY2,position};
    spiSendData(COMMAND_SERVO2,buffer,sizeof(buffer));
    delay(2);//Wait while command is processed
}
void BnrOneA::minBat(float batmin)
{
    int intg=(int)batmin;
    int dec=(int)((batmin-intg)*1000);
    byte intg_H=highByte(intg);
    byte intg_L=lowByte(intg);
    byte dec_H=highByte(dec);
    byte dec_L=lowByte(dec);
    byte buffer[]={KEY1,KEY2,intg_H,intg_L,dec_H,dec_L};
    spiSendData(COMMAND_BAT_MIN,buffer,sizeof(buffer));
    delay(25);//Wait while command is processed
}
void BnrOneA::saveCalibrate(float bat,byte powerL,byte powerR)
{
    int intg=(int)bat;
    int dec=(int)((bat-intg)*1000);
    byte intg_H=highByte(intg);
    byte intg_L=lowByte(intg);
    byte dec_H=highByte(dec);
    byte dec_L=lowByte(dec);
    byte buffer[]={KEY1,KEY2,intg_H,intg_L,dec_H,dec_L,powerL,powerR};
    spiSendData(COMMAND_SAVE_CALIBRATE,buffer,sizeof(buffer));
    delay(35);//Wait while command is processed
}
//////////////////////////////////////////////////////////////////////////////////////
// Read routines
//////////////////////////////////////////////////////////////////////////////////////
byte BnrOneA::readButton()
{
    int adc;
    byte button;
    adc=spiRequestWord(COMMAND_BUT_READ);
    if(adc>=0 && adc<100)	//0-82
    {
      button=1;
    }
    else if(adc>=459 && adc<571)  //509-521
    {
      button=2;
    }
    else if(adc>=629 && adc<737) //679-687
    {
      button=3;
    }
    else
    {
      button=0;
    }
    return button;
}
float BnrOneA::readBattery()
{
   return (float)((float)(spiRequestWord(COMMAND_BAT_READ))/50.7);
}
int BnrOneA::readEncL()
{
    return spiRequestWord(COMMAND_ENCL);
}
int BnrOneA::readEncR()
{
    return spiRequestWord(COMMAND_ENCR);
}
int BnrOneA::readEncLInc()
{
    return spiRequestWord(COMMAND_ENCL_INC);
}
int BnrOneA::readEncRInc()
{
    return spiRequestWord(COMMAND_ENCR_INC);
}
void BnrOneA::readFirmware(byte *firm1,byte *firm2,byte *firm3)
{
    byte value[3]={0,0,0};
    int k=0;
    byte buffer[]={KEY1,KEY2};
    spiSendData(COMMAND_FIRMWARE,buffer,sizeof(buffer));//Request data from master
    digitalWrite(_sspin, LOW); // Select the SPI Slave device to start communication.
    delayMicroseconds(20);
    for (k=0; k<3; k++)
    {
        value[k]=SPI.transfer(0x00);  // Reads one byte
        delayMicroseconds(20);
    }
    *firm1=value[0];
    *firm2=value[1];
    *firm3=value[2];
    digitalWrite(_sspin, HIGH); // Close communication with slave device.
}
byte BnrOneA::obstacleSensors()
{
    return spiRequestByte(COMMAND_OBSTACLES);
}
byte BnrOneA::readIRSensors()
{
    return spiRequestByte(COMMAND_IR_SENSORS);
}
byte BnrOneA::readRangeL()
{
    return spiRequestByte(COMMAND_RANGE_LEFT);
}
byte BnrOneA::readRangeR()
{
    return spiRequestByte(COMMAND_RANGE_RIGHT);
}
int BnrOneA::readAdc(byte channel)
{
  byte command=0x00;
  switch(channel)
  {
      case 0:
          command=COMMAND_ADC0;
          break;
      case 1:
          command=COMMAND_ADC1;
          break;
      case 2:
          command=COMMAND_ADC2;
          break;
      case 3:
          command=COMMAND_ADC3;
          break;
      case 4:
          command=COMMAND_ADC4;
          break;
      case 5:
          command=COMMAND_ADC5;
          break;
      case 6:
          command=COMMAND_ADC6;
          break;
      case 7:
          command=COMMAND_ADC7;
          break;
  }
  return spiRequestWord(command);
}
int BnrOneA::readAdc0()
{
    return spiRequestWord(COMMAND_ADC0);
}
int BnrOneA::readAdc1()
{
    return spiRequestWord(COMMAND_ADC1);
}
int BnrOneA::readAdc2()
{
    return spiRequestWord(COMMAND_ADC2);
}
int BnrOneA::readAdc3()
{
    return spiRequestWord(COMMAND_ADC3);
}
int BnrOneA::readAdc4()
{
    return spiRequestWord(COMMAND_ADC4);
}
int BnrOneA::readAdc5()
{
    return spiRequestWord(COMMAND_ADC5);
}
int BnrOneA::readAdc6()
{
    return spiRequestWord(COMMAND_ADC6);
}
int BnrOneA::readAdc7()
{
    return spiRequestWord(COMMAND_ADC7);
}
int BnrOneA::readDBG(byte index)
{
  byte command=0x00;
  switch(index)
  {
      case 0:
          command=0xB9;
          break;
      case 1:
          command=0xB8;
          break;
      case 2:
          command=0xB7;
          break;
      case 3:
          command=0xB6;
          break;
  }
  return spiRequestWord(command);
}
/**************************************************************/
/**** LCD LINE 1 Handlers *************************************/
/**************************************************************/
void BnrOneA::lcd1(String string)
{   
    int i,a;
    byte buffer[18];
    char string1[19],string2[19];
    for(i=0;i<16;i++){
        string2[i]=string[i];
    }
    string2[16]=0;
    a=sprintf(string1,string2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=' ';
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(byte string[])
{   
    int i,a;
    byte buffer[18];
    char string1[19],string2[19];
    for(i=0;i<16;i++){
        string2[i]=string[i];
    }
    string2[16]=0;
    a=sprintf(string1,string2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=' ';
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(const char string[])
{
    int i,a;
    byte buffer[18];
    char string1[19],string2[19];
    for(i=0;i<16;i++){
        string2[i]=string[i];
    }
    string2[16]=0;
    a=sprintf(string1,"%s",string2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    for(i=a;i<16;i++)
    {
        buffer[i+2]=' ';
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(int number)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%d",number);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(unsigned int number)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%u",number);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(long int number)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%ld",number);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(double number)
{
    int i,a;
    int intg;
	int dec;
    byte buffer[18];
    char string[19];
	bool flag_neg=0;
    
	if (number<-0.0001)
	{
		flag_neg=1;
		number*=-1.0;
	}
	dec = round((number-((double)(int)number))*100.0) % 100;
	intg = (dec==0 ? round(number):(int)number);
    a=sprintf(string,"%d.%02d            ",intg,dec);
	
    buffer[0]=KEY1;
    buffer[1]=KEY2;
	if (flag_neg==1)
		buffer[2]='-';
    for(i=0;i<16;i++){
        buffer[i+2+flag_neg]=string[i];
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(const char string[],int number)
{
    int i, a, b;
    byte buffer[18];
    char string1[19],string2[19];
    for(i=0;i<16;i++){
        string2[i]=string[i];
    }
    string2[16]=0;
    a=sprintf(string1,string2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    a+=2;
    b=sprintf(string1,"%d",number);
    for(i=0;i<b;i++){
        buffer[i+a]=string1[i];
    }
	    for(i=a+b;i<18;i++){
        buffer[i]=(' ');
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(const char string[],unsigned int number)
{
    int i, a, b,c;
    byte buffer[18];
    char string1[19],string2[19];
    for(i=0;i<16;i++){
        string2[i]=string[i];
    }
    string2[16]=0;
    a=sprintf(string1,string2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    a+=2;
    b=sprintf(string1,"%u",number);
    for(i=0;i<b;i++){
        buffer[i+a]=string1[i];
    }
    for(i=a+b;i<18;i++){
        buffer[i]=(' ');
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(const char string[],long int number)
{
    int i, a, b;
    byte buffer[18];
    char string1[19],string2[19];
    for(i=0;i<16;i++){
        string2[i]=string[i];
    }
    string2[16]=0;
    a=sprintf(string1,string2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    a+=2;
    b=sprintf(string1,"%ld",number);
    for(i=0;i<b;i++){
        buffer[i+a]=string1[i];
    }
    for(i=a+b;i<18;i++){
        buffer[i]=(' ');
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(const char string[],double number)
{
    int i, a, b;
    char string1[19];
    char string2[19];
    byte buffer[18];
    int intg, dec;
	bool flag_neg=0;

    for(i=0;i<16;i++){
        string2[i]=string[i];
    }
    string2[16]=0;    
    a=sprintf(string1,string2);    
	buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    a+=2;
	if (number<-0.0001){
		flag_neg=1;
		number*=-1.0;
	}
	dec = round((number-((double)(int)number))*100.0) % 100;
	intg = (dec==0 ? round(number):(int)number);
    b=sprintf(string2,"%d.%02d            ",intg,dec);
	if (flag_neg==1){
		buffer[a]='-';
		a++;
	}		
    for(i=0;i<b;i++){
		if((i+a)<18)
			buffer[i+a]=string2[i];
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(unsigned char stringA[8],unsigned char stringB[8])
{
    int i,a;
    byte buffer[18];
    char string1[17],string2[17];
    for(i=0;i<16;i++){
        if (i<8)
        string2[i]=stringA[i];
        else
        string2[i]=stringB[i-8];
    }
    string2[16]=0;
    a=sprintf(string1,string2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=' ';
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(12);//Wait while command is processed
}
void BnrOneA::lcd1(unsigned int num1, unsigned int num2)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%u %u",num1,num2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(unsigned int num1, unsigned int num2, unsigned int num3)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%u %u %u",num1,num2,num3);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(unsigned int num1, unsigned int num2, unsigned int num3, unsigned int num4)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%4u%4u%4u%4u",num1,num2,num3,num4);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(int num1, int num2)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%d %d",num1,num2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(int num1, int num2, int num3)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%d %d %d",num1,num2,num3);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd1(int num1, int num2, int num3, int num4)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%4d%4d%4d%4d",num1,num2,num3,num4);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L1,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
/**************************************************************/
/**** LCD LINE 2 Handlers *************************************/
/**************************************************************/
void BnrOneA::lcd2(String string)
{
    int i,a;
    byte buffer[18];
    char string1[19],string2[19];
    for(i=0;i<16;i++){
        string2[i]=string[i];
    }
    string2[16]=0;
    a=sprintf(string1,string2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=' ';
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(byte string[])
{
    int i,a;
    byte buffer[18];
    char string1[19],string2[19];
    for(i=0;i<16;i++){
        string2[i]=string[i];
    }
    string2[16]=0;
    a=sprintf(string1,string2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=' ';
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(const char string[])
{
    int i,a;
    byte buffer[18];
    char string1[19],string2[19];
    for(i=0;i<16;i++){
        string2[i]=string[i];
    }
    string2[16]=0;
    a=sprintf(string1,string2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    for(i=a;i<16;i++)
    {
        buffer[i+2]=' ';
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(int number)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%d",number);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(unsigned int number)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%u",number);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(long int number)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%ld",number);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(double number)
{
    int i,a;
    int intg;
	int dec;
    byte buffer[18];
    char string[19];
	bool flag_neg=0;
    
	if (number<-0.0001)
	{
		flag_neg=1;
		number*=-1.0;
	}
	dec = round((number-((double)(int)number))*100.0) % 100;
	intg = (dec==0 ? round(number):(int)number);
    a=sprintf(string,"%d.%02d            ",intg,dec);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
	if (flag_neg==1)
		buffer[2]='-';
    for(i=0;i<16;i++){
        buffer[i+2+flag_neg]=string[i];
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(const char string[],int number)
{
    int i, a, b;
    byte buffer[18];
    char string1[19],string2[19];
    for(i=0;i<16;i++){
        string2[i]=string[i];
    }
    string2[16]=0;
    a=sprintf(string1,string2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    a+=2;
    b=sprintf(string1,"%d",number);
    for(i=0;i<b;i++){
        buffer[i+a]=string1[i];
    }
    for(i=a+b;i<18;i++){
        buffer[i]=(' ');
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(const char string[],unsigned int number)
{
    int i, a, b;
    byte buffer[18];
    char string1[19],string2[19];
    for(i=0;i<16;i++){
        string2[i]=string[i];
    }
    string2[16]=0;
    a=sprintf(string1,string2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    a+=2;
    b=sprintf(string1,"%u",number);
    for(i=0;i<b;i++){
        buffer[i+a]=string1[i];
    }
    for(i=a+b;i<18;i++){
        buffer[i]=(' ');
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(const char string[],long int number)
{
    int i, a, b;
    byte buffer[18];
    char string1[19],string2[19];
    for(i=0;i<16;i++){
        string2[i]=string[i];
    }
    string2[16]=0;
    a=sprintf(string1,string2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    a+=2;
    b=sprintf(string1,"%ld",number);
    for(i=0;i<b;i++){
        buffer[i+a]=string1[i];
    }
    for(i=a+b;i<18;i++){
        buffer[i]=(' ');
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(const char string[],double number)
{
    int i, a, b;
    char string1[19];
    char string2[19];
    byte buffer[18];
    int intg, dec;
	bool flag_neg=0;

    for(i=0;i<16;i++){
        string2[i]=string[i];
    }
    string2[16]=0;    
    a=sprintf(string1,string2);
	buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    a+=2;
	if (number<-0.0001){
		flag_neg=1;
		number*=-1.0;
	}
	dec = round((number-((double)(int)number))*100.0) % 100;
	intg = (dec==0 ? round(number):(int)number);
    b=sprintf(string2,"%d.%02d            ",intg,dec);
	if (flag_neg==1){
		buffer[a]='-';
		a++;
	}		
    for(i=0;i<b;i++){
		if((i+a)<18)
			buffer[i+a]=string2[i];
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(unsigned char stringA[] ,unsigned char stringB[])
{
    int i,a;
    byte buffer[18];
    char string1[17],string2[17];
    for(i=0;i<16;i++){
        if (i<8)
        string2[i]=stringA[i];
        else
        string2[i]=stringB[i-8];
    }
    string2[16]=0;
    a=sprintf(string1,string2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<a;i++){
        buffer[i+2]=string1[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=' ';
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(12);//Wait while command is processed
}
void BnrOneA::lcd2(unsigned int num1, unsigned int num2)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%u %u",num1,num2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(unsigned int num1, unsigned int num2, unsigned int num3)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%u %u %u",num1,num2,num3);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(unsigned int num1, unsigned int num2, unsigned int num3, unsigned int num4)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%4u%4u%4u%4u",num1,num2,num3,num4);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(int num1, int num2)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%d %d",num1,num2);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(int num1, int num2, int num3)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%d %d %d",num1,num2,num3);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}
void BnrOneA::lcd2(int num1, int num2, int num3, int num4)
{
    int i, a=0;
    byte buffer[18];
    char string[17];
    a=sprintf(string,"%4d%4d%4d%4d",num1,num2,num3,num4);
    buffer[0]=KEY1;
    buffer[1]=KEY2;
    for(i=0;i<16;i++){
        buffer[i+2]=string[i];
    }
    for(i=a;i<16;i++){
        buffer[i+2]=(' ');
    }
    spiSendData(COMMAND_LCD_L2,buffer,sizeof(buffer));
    delay(4);//Wait while command is processed
}

/***********************************************************************************************/
int BnrOneA::readLine()
{	
	  #define VMAX 1000
	  static int SValMax[8]={1023,1023,1023,1023,1023,1023,1023,1023};
	  static int SValMin[8]={0,0,0,0,0,0,0,0};
	  static double SFact[8];
	  static int Vtrans=50;  //
	  static bool loadFlag=0;

	  int Vrt1=SValMin[1]*2, Vrt2=SValMin[6]*2;
      int SValR[8];  
      int SValN[10]={Vrt1,0,0,0,0,0,0,0,0,Vrt2};    
      int idMax=-1, SMax=-1;
      int lineValue=-1;
      int flag=-1;
      static int prevLineValue=0;

	  if(loadFlag==0)
	  {
		   //Ler valores da EEPROM
		   byte eepromADD=100;
		   for(int i=0;i<8;i++)
		   {
			   SValMax[i]=(int)EEPROM.read(eepromADD);
			   SValMax[i]=SValMax[i]<<8;
			   eepromADD++;
			   SValMax[i]+=(int)EEPROM.read(eepromADD);
			   eepromADD++;
		   }
		   for(int i=0;i<8;i++)
		   {
			   SValMin[i]=(int)EEPROM.read(eepromADD);
			   SValMin[i]=SValMin[i]<<8;
			   eepromADD++;
			   SValMin[i]+=(int)EEPROM.read(eepromADD);
			   eepromADD++;
		   }   
		   Vtrans=(int)EEPROM.read(eepromADD);
		   Vtrans=Vtrans<<8;
		   eepromADD++;
		   Vtrans+=(int)EEPROM.read(eepromADD);
		   
		   for(int i=0;i<8;i++)
		   {
			  SFact[i]=(double)VMAX/(double)(SValMax[i]-SValMin[i]); //Calcular fator de cada sensor
		   }
		   loadFlag=1;
	  }

      //Leitura dos valores dos 8 sensores
      for(int i=0;i<8;i++)
      {
          SValR[i]=readAdc(i);
      }

      //Normalizar valores entre 0 e 1000
      for(int i=1;i<9;i++)
      {
          SValN[i]=(int)((double)((SValR[i-1]-SValMin[i-1]))*SFact[i-1]); //Registar o valor efetivo m�ximo de cada sensor
          if(SValN[i]>SMax)
            {
              SMax=SValN[i]; //Identificar o sensor com valor efectivo m�ximo
              idMax=i;      //Registar o indice do sensor
            }
      }
      
      if(SMax>Vtrans && SValN[idMax-1]>=SValN[idMax+1]) //Se o anterior for maior que o seguinte
      {
          lineValue=VMAX*(idMax-1)+SValN[idMax];     
             flag=0;
      }
      else if(SMax>Vtrans && SValN[idMax-1]<SValN[idMax+1]) //Se o anterior for menor que o seguinte
      {
          if(idMax!=8) // Se n�o � o �ltimo sensor
          {
             lineValue=VMAX*idMax+SValN[idMax+1];
             flag=1;
          }
          else //Se � o �ltimo sensor
          {
             lineValue=VMAX*idMax+VMAX-SValN[idMax];
             flag=2;
          }
      }

      if(lineValue==-1)//sa�u da linha -> tudo branco
      {
        if(prevLineValue>4500)
        {
          lineValue=9000;
        }
        else
        {
          lineValue=0;
        }
      }      
      else if(lineValue<-1 || lineValue>9000) //Possiveis erros de leitura
      {
        lineValue=prevLineValue;
      }
      else //se valores normais
      {
        prevLineValue=lineValue;
      }
//      return lineValue;  // Valores de 0 a 9000
      return (int)((double)(lineValue+1)*0.022222)-100;  // Valores de -100 a 100
}

/***********************************************************************************************/
