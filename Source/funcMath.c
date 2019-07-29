#include "ti/lib_aux/funcMath.h"

float absolute(float value){
    if(value < 0){
        return -1 * value;
    }else{
        return value;
    }
}

int round(float value){
    int l = value;
    int u = l + 1;

    if(absolute(value - l) < absolute(value - u)){
        return l;
    }else{
        return u;
    }
}

uint32_t min(uint32_t x, uint32_t y){
    if(x < y){
        return x;
    }else{
        return y;
    }
}
