#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>      /* printf */
#include <math.h>
using namespace std;

//Functions:
//--------------------------------------------------------------------------------------------------------------------
// radian to degree
double rtod(double rad){
    double deg = 0;
    deg = rad * 57.2958;
return deg;
}
// degree to radian
double dtor(double deg){
    double rad = 0;
    rad = deg / 57.2958;
return rad;
}

//gets angle between two points, returns angle in rad.
double getAngle(double first[3], double second[3]){
    double angle=0; // return value
    //[0] x [1] y
    // calculate angle
    angle= atan((second[0]-first[0])/(second[1]-first[1]));
    //error code when points on top of eachother
    if(first[0]==second[0]&&first[1]==second[1]){
        angle= -999;
        errorCode=1;
    }
    return angle;
}

// read out errorCode number
int errorReadOut(int errorCode){
    // "if" may not be neccisary (for i<0)
    if(errorCode==0){
        return 0;
    }
    else{
        unsigned int refTime=0;
        //refTime=millis();
        for(int i=0; i<errorCode; i++){
            while(refTime<refTime+1000){
                //digitalWrite(30,HIGH);
            }
            //refTime=millis();
            while(refTime<refTime+1000){
                //digitalWrite(30,LOW);
            }
        }
        return 1;
    }
}



/*
// main test function ----------------
int main(){
    double one[2]={1.5,0};
    double two[2]={1,1};
    cout<< "(" << two[0] << " - " << one[0]<<" )/( " << two[1] << " - " << one[1] << " )" << endl; 
    //(second[0]-first[0])/(second[0]-first[1])

    cout<< "Angle: "<< rtod(getAngle(one,two))<<endl;
}
*/