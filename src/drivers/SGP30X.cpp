// =============================
// File: drivers/SGP30X.cpp
// =============================
#include "SGP30X.h"


bool SGP30X::begin(){ 
	if(!_sgp.begin()) return false; _sgp.IAQinit(); return true; 
}

bool SGP30X::read(uint16_t &eco2, uint16_t &tvoc){ 
	if(!_sgp.IAQmeasure()) return false; eco2=_sgp.eCO2; tvoc=_sgp.TVOC; return true; 
}