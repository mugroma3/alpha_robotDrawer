#include "Arduino.h"

template<int Pin>
struct Button{
    int last_state {LOW};
    Button(){ pinMode(Pin, INPUT_PULLUP);}
    bool isRisingEdge(){ 
        int state=digitalRead(Pin);
        bool rising=last_state==LOW && state==HIGH; 
        last_state=state;
        return rising;
    }
};
