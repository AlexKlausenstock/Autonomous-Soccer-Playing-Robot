//Outputs Velo to move towards cordinates
//#include <VeloToMotor.cpp>
//#include <Variables.cpp>

#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>      /* printf */
#include <math.h>

#include <Variables.cpp>

using namespace std;

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

// robot x y theta (0 = pointing +y) clock wise is +
//enemy robot xy
//ball xy
//destination x y theta (make just xy version?) how get theta that will never happen?

double robot[3]={0,0,45};
double enemy[2]={0,0};
double ball[2]={0,0};
double ballVelo[2]={0,0};

double destination[3]={0,0,0};
double dtranslation[3]={0,0,0};

//from VeloToMotor variables
double velo[3]={0,0,0};
double motor[4]= {0,0,0,0};
double norm = 0;


int main(){
cout<<"Input direction x y theta"<<endl;
cin>>destination[0];
cin>>destination[1];
cin>>destination[2];

//----------------------------------------------------------------------------------------------------
// Go towards x y theta

// find dx and dy
for (int i=0; i<2;i++){
    dtranslation[i]=destination[i]-robot[i];
    cout<<"dtranslation: " << dtranslation[i] << endl;
}
// find dtheta
    dtranslation[2]=destination[2] - robot[2];
    cout<<"dtranslation: " << dtranslation[2] << endl;

// convert to x and y to x,y prime 
//NOTE (-) is added to to bot angle relative to court because the sourced equations use a counter clock wise is (+) sense, I use clock wise is (+) sense
//https://en.wikipedia.org/wiki/Rotation_of_axes

//y
velo[1]=-(dtranslation[0]*sin(dtor(-robot[2])))+(dtranslation[1]*cos(dtor(-robot[2])));
//x
velo[0]=((dtranslation[0]*cos(dtor(-robot[2])))+(dtranslation[1]*sin((dtor(-robot[2])))));
//theta
velo[2]=dtranslation[2]/4; // /4 is to  reduce scaling of degrees vs translation units of probably inches or cm


cout << "velo_x: " << velo[0]<<endl;
cout << "velo_y: " << velo[1]<<endl;
cout << "vel_theta: " << velo[2] << endl;
cout<< "robot angle: " << robot[2]<<endl;
// normalize translation?


}

// The drive function will normalize this
// how will it normalize rotation
// when input roation set it simmilar scale to max component? (it's fine it will get refreshed) (fix later if it is fucked)
// self normalize before storing? where max velues are 255?


/*
// example normalizing code **************************************
// make sure all motor values are under 255 (max input value) by scaling by max value of motor
    // normm = max element of motor
    for(int i=0; i<=3;i++){
        if(abs(motor[i])>norm){
            norm=abs(motor[i]);
        }
    }
    //divide all elements by norm to make motor array into unit vector then multiply by max value 255 to assure
    if(norm!=0 && norm>255){
        for(int i=0; i<=3;i++){
            motor[i]=motor[i]/norm;
            motor[i]=motor[i]*255;
        }
    }
    //round motor values to be an integer(floor=round down)
    for(int i=0; i<=3;i++){
        motor[i]=floor(motor[i]);
    }
// END normalizing code**************************************
*/