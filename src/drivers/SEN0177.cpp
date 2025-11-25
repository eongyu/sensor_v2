// =============================
// File: drivers/SEN0177.cpp
// =============================
#include "SEN0177.h"


bool SEN0177::begin(HardwareSerial &ser, int rxPin, int txPin, uint32_t baud){
	_ser=&ser; _ser->setRxBufferSize(512); 
	_ser->begin(baud, SERIAL_8N1, rxPin, txPin); 
	_ser->setTimeout(50);
	while(_ser->available()) _ser->read(); 
	
	return true;
}


bool SEN0177::read(PM25Data &data){
	if(!_ser) return false;
	while(_ser->available()>=32){
		if(_ser->read()!=0x42) continue; if(_ser->peek()!=0x4D) continue; _ser->read();
		uint8_t buf[30]; if(_ser->readBytes(buf, sizeof(buf))!=sizeof(buf)) return false;
		uint16_t len=(buf[0]<<8)|buf[1]; if(len!=28) continue; uint16_t cs_rx=(buf[28]<<8)|buf[29];
		uint32_t sum=0x42+0x4D; for(int i=0;i<28;i++) sum+=buf[i]; if(((uint16_t)sum)!=cs_rx) continue;
		data.pm1_0=(buf[2]<<8)|buf[3]; data.pm2_5=(buf[4]<<8)|buf[5]; data.pm10=(buf[6]<<8)|buf[7]; return true;
	}
	return false;
}