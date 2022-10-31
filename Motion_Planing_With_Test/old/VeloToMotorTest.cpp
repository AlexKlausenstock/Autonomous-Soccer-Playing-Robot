// Normalization stuff for driver code
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>      /* printf */
#include <math.h>

using namespace std;
int main(){
    // velo [0] (front and rear wheels towards and away from eachother) // +x direction
    // velo[1] (all in 1 direction) // +y dir forward*
    // velo[2] (left weels + right wheels -) // rotation clockwise
    // motor control motors[4] Index: (clockwise top left first order) 

    
    //1 get desired direction in x y and rotation and convert to motor values via supper position
    //2 normalize to limit max motor vectors to 255
    //3 Order motors to move

    double velo[3]={0,0,0};
    double motor[4]= {0,0,0,0};
    double norm = 0;

    //input [0] x axis [1] y axis [2] rotation clockwise
    // for testing code
    /*
    velo[0]=0;
    velo[1]=0;
    velo[2]=0;
    */

    // eliminate very small motion to creat motion dead zone (because it will normalize to max 255)
    for(int i=0; i<3;i++){
        if (velo[i]<.8){
            velo[i]=0;
        }
    }

    // find motor values
    //front left
    motor[0]=velo[1]+velo[0]+velo[2];
    //front right
    motor[1]=velo[1]-velo[0]-velo[2];
    //back right
    motor[2]=velo[1]+velo[0]-velo[2];
    //back left
    motor[3]=velo[1]-velo[0]+velo[2];

    // debug pre normalized read out
    for(int i=0; i<=3;i++){
        cout << "motor[" << i << "] " << motor[i]<< endl;
    }

    // normalize by the max motor value
    // normm = max element of motor
    for(int i=0; i<=3;i++){
        if(abs(motor[i])>norm){
            norm=abs(motor[i]);
        }
    }
    cout<<"norm: " << norm << endl;
    if(norm!=0 && norm> MaxSpeed){
        for(int i=0; i<=3;i++){
            motor[i]=motor[i]/norm;
            motor[i]=motor[i]* MaxSpeed;
        }
    }
    // Replace 255 with MaxSpeed value and re test ----------------------------------------------------------------------------

    // round to integer for pwm value
    for(int i=0; i<=3;i++){
        motor[i]=floor(motor[i]);
        cout << "motorfloor[" << i << "] " << motor[i]<< endl;
    }
    // call spinMotor function with these motor functions in arduino
    
}
