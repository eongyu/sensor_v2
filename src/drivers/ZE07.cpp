// =============================
// File: drivers/ZE07.cpp
// =============================
#include "ZE07.h"


bool ZE07::begin(HardwareSerial &ser, int rxPin, int txPin, uint32_t baud){
	_ser=&ser; _ser->begin(baud, SERIAL_8N1, rxPin, txPin); return true;
}


void ZE07::setQA(bool qa_mode){
	static const uint8_t CMD_QA[9] = {0xFF,0x01,0x78,0x41,0x00,0x00,0x00,0x00,0x46};
	static const uint8_t CMD_AUTO[9] = {0xFF,0x01,0x78,0x40,0x00,0x00,0x00,0x00,0x47};
	_qa = qa_mode; if(!_ser) return; _ser->write(_qa?CMD_QA:CMD_AUTO, 9); _ser->flush();
}


bool ZE07::read_frame(uint32_t timeout_ms){
	if(!_ser) return false; uint32_t st=millis();

	while(millis()-st < timeout_ms){
		if(_ser->available()){
			uint8_t b=(uint8_t)_ser->read(); if(b!=0xFF) continue; _frame[0]=b;
			int idx=1; while(idx<9 && (millis()-st<timeout_ms)) if(_ser->available()) _frame[idx++]=(uint8_t)_ser->read();
			if(idx==9){ return checksum_(_frame)==_frame[8]; }
			break;
		}
	}
	return false;
}


bool ZE07::parse_ppm(float &ppm, uint16_t &full, uint8_t &dec) const{
	if(_frame[0]!=0xFF) return false; uint8_t h=_frame[4], l=_frame[5]; uint8_t fh=_frame[6], fl=_frame[7]; dec=_frame[3];
	uint16_t conc=((uint16_t)h<<8)|l; ppm=(float)conc*0.1f; full=((uint16_t)fh<<8)|fl; return true;
}