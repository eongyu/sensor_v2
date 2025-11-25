// =============================
// File: core/LEDs.cpp
// =============================
#include "LEDs.h"


namespace leds {
    void init(){
        pinMode(PIN_LED1, OUTPUT);
        pinMode(PIN_LED2, OUTPUT);
        pinMode(PIN_LED3, OUTPUT);
        pinMode(PIN_LED4, OUTPUT);

        digitalWrite(PIN_LED1, LOW);
        digitalWrite(PIN_LED2, LOW);
        digitalWrite(PIN_LED3, LOW);
        digitalWrite(PIN_LED4, LOW);
    }

    void blink1(bool on){ digitalWrite(PIN_LED1, on?HIGH:LOW); }
    void blink2(bool on){ digitalWrite(PIN_LED2, on?HIGH:LOW); }
    void blink3(bool on){ digitalWrite(PIN_LED3, on?HIGH:LOW); }
    void blink4(bool on){ digitalWrite(PIN_LED4, on?HIGH:LOW); }
    
    void set1(bool on){ digitalWrite(PIN_LED1, on?HIGH:LOW); }
    void set2(bool on){ digitalWrite(PIN_LED2, on?HIGH:LOW); }
    void set3(bool on){ digitalWrite(PIN_LED3, on?HIGH:LOW); }
    void set4(bool on){ digitalWrite(PIN_LED4, on?HIGH:LOW); }
}