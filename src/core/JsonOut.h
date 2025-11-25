// =============================
// File: core/JsonOut.h
// =============================
#pragma once
#include <Arduino.h>


class JsonOut {
    public:
        JsonOut(Stream &s): _s(s), _first(true) { _s.print("{ "); }
        ~JsonOut(){ _s.println(" }"); }
        void add(const char* k, float v, uint8_t digits=2){ key(k); _s.print(v, digits);}
        void addU(const char* k, uint32_t v){ key(k); _s.print(v);}
        void addS(const char* k, const char* v){ key(k); _s.print('"'); _s.print(v); _s.print('"');}
    
    private:
        void key(const char* k){ if(!_first) _s.print(","); _first=false; _s.print('"'); _s.print(k); _s.print('"'); _s.print(":"); }
        Stream &_s; bool _first;
};