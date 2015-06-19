#include <math.h>

class Walker{

    Servo sxservo, dxservo;
    float time_action;
    bool acting {false};
    long last_time;

    static constexpr float maxvel = 0.000153;              //cm * ms^-1
    static constexpr float r = 0.153 / 2;               //(distance between wheels)/2,  cm
    static constexpr float compensation=1.19;           //wut?
    static constexpr float r_maxvel=(r*compensation)/maxvel;    //inverse of angular velocity of the spinning craft, ms * rad^-1

    static constexpr int dx_minEndpoint = 1250; //1000
    static constexpr int dx_maxEndpoint = 1750; 

    static constexpr int sx_minEndpoint = 1250; //1000
    static constexpr int sx_maxEndpoint = 1750; 

    int servoMap_us(float val, int minEndpoint, int maxEndpoint){
        return (int)((val + 1) * (maxEndpoint - minEndpoint)) / 2 + minEndpoint;
    }

    void sx(float val){
        sxservo.writeMicroseconds(servoMap_us(-val, sx_minEndpoint, sx_maxEndpoint));
    }

    void dx(float val){
        dxservo.writeMicroseconds(servoMap_us(val, dx_minEndpoint, dx_maxEndpoint));
    }

    void setTimeAction(float t){
        time_action=t;
        last_time=millis();
        acting=true;
    }
public:

    Walker(int Sx, int Dx) {
        dxservo.attach(Dx);
        sxservo.attach(Sx);
        stop();
    }

    bool update(){
        if(acting && millis()-last_time >= (time_action)){
            acting=false; stop();
        }
        return acting;
    }

    void rotate(float angle){
        if(angle==0){stop(); return;}

        setTimeAction(fabs(angle*r_maxvel));

        if(angle>0){
            sx(1);
            dx(-1);
        }else{
            sx(-1);
            dx(1);
        }
    }

    void move(float mag){
        if(mag==0){stop(); return;}

        setTimeAction(mag/maxvel);

        sx(1);
        dx(1);
    }


    void stop(){
        sx(0);
        dx(0);
    }
};
