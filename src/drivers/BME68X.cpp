// =============================
// File: drivers/BME68X.cpp
// =============================
#include "BME68X.h"


bool BME68X::begin(uint8_t addr){
	if(!_bme.begin(addr)) return false;

	_bme.setTemperatureOversampling(BME680_OS_8X);
	_bme.setHumidityOversampling(BME680_OS_2X);
	_bme.setPressureOversampling(BME680_OS_4X);
	_bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
	_bme.setGasHeater(320,150);

	return true;
}


bool BME68X::read(float &t, float &h, float &g){
	if(!_bme.performReading()) 
		return false; 
	
	t=_bme.temperature; 
	h=_bme.humidity; 
	_last_gas_res = g = _bme.gas_resistance;
	
	return true;
}