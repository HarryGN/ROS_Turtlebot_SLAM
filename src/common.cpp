#include "common.h"




float absPow(float base, float exp){
    if(base < 0){
        return (float) -1*pow(-1*base, exp);
    }
    else{
        return (float) pow(base, exp);
    } 
}

void applyMagnitudeLimits(float &value, float lowerLimit, float upperLimit){
    if(value < 0){
        if(value < -upperLimit){
            value = -upperLimit;
        }
        else if(value > -lowerLimit){
            value = -lowerLimit;
        }
    }

    else if(value > 0){
        if(value > upperLimit){
            value = upperLimit;
        }
        else if(value < lowerLimit){
            value = lowerLimit;
        }
    }
}

float distanceBetween(float x1, float y1, float x2, float y2){
    return sqrt(pow((x1-x2),2) + pow((y1-y2),2));
}

void wrapIntegerIndexAroundRange(int &index, int start, int end){
    int range = end-start + 1;
    while(index > end){
        index -= range; 
    }
    
    while(index < start){
        index += range;
    }
}