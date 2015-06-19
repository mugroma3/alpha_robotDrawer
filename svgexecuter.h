#include <stdlib.h>
#include <math.h>

namespace {
    struct PolarVec { float mag, angle; };

    struct CartesianVec{ 
        float x, y;
        operator PolarVec() const{
            float mag=lenght();
            if(mag==0) return {0, 0};

            float angle = atan2(y, x);
            if(angle<0){
                angle=2*M_PI + angle;
            }

            return {mag, angle};
        }

        float lenght() const{
            return sqrt(x*x+y*y);
        }

        CartesianVec& operator+=(CartesianVec c){
            x+=c.x; y+=c.y; return *this;
        }

        CartesianVec& operator-=(CartesianVec c){
            x-=c.x; y-=c.y; return *this;
        }

    };

    CartesianVec operator+(CartesianVec a, CartesianVec b){return a+=b;}
    CartesianVec operator-(CartesianVec a, CartesianVec b){return a-=b;}
    CartesianVec operator-(CartesianVec a){return {-a.x, -a.y};}
    
    float angleDifference(float angle_from, float angle_to){
        float res=angle_to-angle_from;
        if(res>M_PI){
            res-=2*M_PI;
        }else if(res<-M_PI){
            res+=2*M_PI;
        }

        return res;
    }

}

class SvgExecuter{
    Walker* walker;
    char* path;
    int current{0};

    bool pathDecodingComplete {false};

    CartesianVec position{0, 0};
    float heading{0};

    constexpr static int instrQueueLenght {4};

    enum class LowLevelOp {NONE, MOVE, ROTATE};
    struct LowLevelInstruction {
        LowLevelOp op;
        union{
            float mag;
            float angle;
        } arg;
    };

    LowLevelInstruction instrQueue[instrQueueLenght];
    int instrQueueEnd{0};

    char readCommandFromPath(){
        int i=current;
        while(path[i]!='\0' && !(path[i]>='A' && path[i]<='Z')){
            ++i;
        }
        char res=path[i];
        
        current=i+1;
        return res;
    }

    float extractParamFromPath(){
        int i=current;
        //consume white spaces
        while(path[i]!='\0' && path[i]!='.' && !(path[i]>='0' && path[i]<='9')){
            i++;
        }
        if(path[i]=='\0'){
            current=0;
            return -1;
        }

        //convert number
        float val=atof(&(path[i]));
        
        //consume number
        while(path[i]=='.' || (path[i]>='0' && path[i]<='9')){
            i++;
        }
            
        current=i;
        return val;
    }


    void pushGoTo(CartesianVec relativeMove){
        PolarVec polar= PolarVec(relativeMove);
        if(polar.mag<=epsilon) return;

        instrQueue[instrQueueEnd] = {LowLevelOp::ROTATE, angleDifference(heading, polar.angle)};
        ++instrQueueEnd;

        instrQueue[instrQueueEnd] = {LowLevelOp::MOVE, polar.mag};
        ++instrQueueEnd;

        position+=relativeMove;
        heading=polar.angle;
    }

    void pushLineTo(){
        CartesianVec g;
        g.x = extractParamFromPath();
        g.y = -extractParamFromPath();
        pushGoTo(g-position);
    };

    constexpr static float epsilon {0.1f}; 
    void pushHome(){
        pushGoTo(-position);

        instrQueue[instrQueueEnd] = {LowLevelOp::ROTATE, angleDifference(heading, 0)};
        ++instrQueueEnd;
        heading=0;
        pathDecodingComplete=true;
    }

    CartesianVec subPathStart{0,0};

    void pushMoveTo(){
        float x, y;
        x = extractParamFromPath();
        y = -extractParamFromPath();
        subPathStart={ x, y};
        pushGoTo(subPathStart - position);
    }
    void pushClosePath(){
        pushGoTo(subPathStart-position);
    }

    void pushHorizontalLineTo(){
        CartesianVec h;
        h.x = extractParamFromPath();
        h.y = position.y;
        pushGoTo(h-position);
    }

    void pushVerticalLineTo(){
        CartesianVec v;
        v.x = position.x;
        v.y = -extractParamFromPath();
        pushGoTo(v-position);
    }

    enum class Curve {NONE, CUBIC_BEZIER};

    Curve curveToComplete {Curve::NONE};

    CartesianVec curveStart;
    CartesianVec curveControl1;
    CartesianVec curveControl2;
    CartesianVec curveTarget;

    int curve_subdivisions {};
    int curve_step {};
    void pushCubicBezierTo(){
        if(curveToComplete==Curve::NONE){
            curveToComplete=Curve::CUBIC_BEZIER;
            curveStart=position;
            curveControl1.x=extractParamFromPath();
            curveControl1.y=-extractParamFromPath();
            curveControl2.x=extractParamFromPath();
            curveControl2.y=-extractParamFromPath();
            curveTarget.x=extractParamFromPath();
            curveTarget.y=-extractParamFromPath();

            float lenghtEstimation= (curveTarget-curveControl2).lenght()    +
                                    (curveControl2-curveControl1).lenght()  +
                                    (curveControl1-curveStart).lenght();
            curve_subdivisions=int(ceil(lenghtEstimation/2));
            curve_step=1;
        }

        if(curve_step<curve_subdivisions){
            float t = ((float) curve_step) / curve_subdivisions;
            float tt = t * t;
            float ttt = tt * t;
            float u = 1 - t;
            float uu = u * u;
            float uuu = uu * u;

            CartesianVec inter;
            inter.x = uuu * curveStart.x;
            inter.x += 3 * uu * t * curveControl1.x;
            inter.x += 3 * u * tt * curveControl2.x;
            inter.x += ttt * curveTarget.x;

            inter.y = uuu * curveStart.y;
            inter.y += 3 * uu * t * curveControl1.y;
            inter.y += 3 * u * tt * curveControl2.y;
            inter.y += ttt * curveTarget.y;

            ++curve_step;
            pushGoTo(inter-position);
        }else{
            curveToComplete=Curve::NONE;
            pushGoTo(curveTarget-position);
        }
    }

public:
    SvgExecuter(Walker* w, char* p): walker{w}, path{p} {};

    // non blocking pilot, return true if there is still work to do
    bool update(){
        //wait till the completition of a move
        if(walker->update()) return true;

        //else
        //execute commands in the buffer
        if(instrQueueEnd!=0){
            //excute command
            switch(instrQueue[0].op){
                case LowLevelOp::MOVE:
                    walker->move(instrQueue[0].arg.mag);
                break;
                case LowLevelOp::ROTATE:
                    walker->rotate(instrQueue[0].arg.angle);
                break;
                default:
                break;
            }
            //shift left command buffer
            for(int i=1; i<instrQueueLenght; ++i){
                if(i==instrQueueEnd) break;
                instrQueue[i-1]=instrQueue[i];
            }
            instrQueueEnd--;

            return true;
        }

        //else
        //continue executing curve command
        if(curveToComplete!=Curve::NONE){
            switch(curveToComplete){
                case Curve::CUBIC_BEZIER:
                    pushCubicBezierTo();
                    break;
                default:
                    curveToComplete=Curve::NONE;
                    break;
            }

            return true;
        }

        //else
        //try to load new commands in the buffer
        if(!pathDecodingComplete){
            switch(readCommandFromPath()){
                case 'M':
                    pushMoveTo();
                    break;
                case 'L':
                    pushLineTo();
                    break;
                case 'H':
                    pushHorizontalLineTo();
                    break;
                case 'V':
                    pushVerticalLineTo();
                    break;
                case 'C':
                    pushCubicBezierTo();
                    break;
                case 'Z':
                    pushClosePath();
                    break;
                case '\0':
                default:
                    pushHome();
                    break;
            }
            return true;
        }

        //really, no more to do
        return false;
    }

    void stop(){
        walker->stop();
    }
};
