#include "JY901.h"
#include "string.h"

JY901::JY901(unsigned char ucAddr)
{
	ucDevAddr = ucAddr;
	tapDetector = TapDetector(1, [](){}); // Tripple tap
}
unsigned char JY901::StartIIC(unsigned char ucAddr)
{
	ucDevAddr = ucAddr;
	delay(10);
	GetTemp();
	if(1000<stcAngle.Temp&stcAngle.Temp<10000)return 1;
	else return 0;
}
void JY901 ::CopeSerialData(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55) 
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}
	else
	{
		switch(ucRxBuffer[1])
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
			case 0x59:	memcpy(&stcQuater,&ucRxBuffer[2],8);break;
			case 0x5a:	memcpy(&stcSN,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;
	}
}
void JY901::readRegisters(unsigned char deviceAddr,unsigned char addressToRead, unsigned char bytesToRead, char * dest)
{
  Wire.beginTransmission(deviceAddr);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(deviceAddr, bytesToRead); //Ask for bytes, once done, bus is released by default

  while(Wire.available() < bytesToRead); //Hang out until we get the # of bytes we expect

  for(int x = 0 ; x < bytesToRead ; x++)
    dest[x] = Wire.read();    
}
void JY901::writeRegister(unsigned char deviceAddr,unsigned char addressToWrite,unsigned char bytesToRead, char *dataToWrite)
{
  Wire.beginTransmission(deviceAddr);
  Wire.write(addressToWrite);
  for(int i = 0 ; i < bytesToRead ; i++)
  Wire.write(dataToWrite[i]);
  Wire.endTransmission(); //Stop transmitting
}

short JY901::ReadWord(unsigned char ucAddr)
{
	short sResult;
	readRegisters(ucDevAddr, ucAddr, 2, (char *)&sResult);
	return sResult;
}
void JY901::WriteWord(unsigned char ucAddr,short sData)
{	
	writeRegister(ucDevAddr, ucAddr, 2, (char *)&sData);
}
void JY901::ReadData(unsigned char ucAddr,unsigned char ucLength,char chrData[])
{
	readRegisters(ucDevAddr, ucAddr, ucLength, chrData);
}
void JY901::GetTime()
{
	readRegisters(ucDevAddr, YYMM, 8, (char*)&stcTime);	
}
void JY901::GetAcc()
{
	readRegisters(ucDevAddr, AX, 6, (char *)&stcAcc);
}
void JY901::GetGyro()
{
	readRegisters(ucDevAddr, GX, 6, (char *)&stcGyro);
}
void JY901::GetAngle()
{
	readRegisters(ucDevAddr, Roll, 6, (char *)&stcAngle);
}
void JY901::GetMag()
{
	readRegisters(ucDevAddr, HX, 6, (char *)&stcMag);
}
void JY901::GetPress()
{
	readRegisters(ucDevAddr, PressureL, 8, (char *)&stcPress);
}
void JY901::GetDStatus()
{
	readRegisters(ucDevAddr, D0Status, 8, (char *)&stcDStatus);
}
void JY901::GetLonLat()
{
	readRegisters(ucDevAddr, LonL, 8, (char *)&stcLonLat);
}
void JY901::GetGPSV()
{
	readRegisters(ucDevAddr, GPSHeight, 8, (char *)&stcGPSV);
}
void JY901::GetQuater()
{
	readRegisters(ucDevAddr, Q0, 8, (char *)&stcQuater);
}
void JY901::GetTemp()
{
	readRegisters(ucDevAddr, TEMP, 2, (char *)&stcAngle.Temp);
}
bool JY901::GetTapDetector()
{
	GetAcc();
	return tapDetector.update(stcAcc.a[1]/32768*16);
}
void JY901::Unlock()
{
	writeRegister(ucDevAddr,0x69,2,(char *)0x88B5);
}
void JY901::Save(unsigned char save)
{
	short td=save<<8|0x00;
	writeRegister(ucDevAddr,SAVE,2,(char *)td);
}
void JY901::SetDirection(unsigned char dir)
{
	short td=dir<<8|0x00;
	writeRegister(ucDevAddr,DIRECTION,2,(char *)td);

}
void JY901::SetALG(unsigned char alg)
{
	short td=alg<<8|0x00;
	writeRegister(ucDevAddr,ALG,2,(char *)td);
}
void JY901::SetCalsw(unsigned char cal)
{
	short td=cal<<8|0x00;
	writeRegister(ucDevAddr,CALSW,2,(char *)td);
}