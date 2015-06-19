#include <Servo.h>

#include "button.h"
#include "walker.h"
#include "svgexecuter.h"

void setup(){
    char* path = "M 10,10 L 20,10 L 20,20 L 10,20 L 10,10";


    Walker walker(9, 10);

    SvgExecuter executer{&walker, path};

    
    Button<3> startNstop;
    bool start = false;
     
    for(;;){
        if(startNstop.isRisingEdge()){
            start = !start;
        }

        if(start){
            executer.update();
        }else{
            executer.stop();
        }
    }

}

void loop(){}



