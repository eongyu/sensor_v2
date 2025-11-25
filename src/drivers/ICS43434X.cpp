// =============================
// File: drivers/ICS43434X.cpp
// =============================
#include "ICS43434X.h"
#include <math.h>


bool ICS43434X::begin(int bclk, int lrck, int din, int fs){
    i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = fs,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    #if ESP_IDF_VERSION_MAJOR >= 4
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    #else
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    #endif
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
    };
    i2s_pin_config_t pins = { .bck_io_num=bclk, .ws_io_num=lrck, .data_out_num=-1, .data_in_num=din };
    if(i2s_driver_install(_port, &cfg, 0, NULL)!=ESP_OK) return false; i2s_set_pin(_port, &pins); i2s_zero_dma_buffer(_port); return true;
}


float ICS43434X::read_rms(){
    int32_t buf[1024]; 
    size_t br=0; 
    float acc=0; 
    
    i2s_read(_port, (void*)buf, sizeof(buf), &br, portMAX_DELAY); 
    size_t n=br/sizeof(int32_t); if(!n) return 0; 
    for(size_t i=0;i<n;i++){ 
        float v=(float)(buf[i]>>8); 
        acc+=v*v;
    } 
    return sqrtf(acc/n);
}