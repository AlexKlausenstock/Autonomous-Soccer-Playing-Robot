//Filter Incorrect Radio function
//use robot cordinates xy and ball xy, compare to robot vs ball angle and dist to see if realistic

//get angle from camera
double angle = 0;

//if camera code is realistic
void setRobot2(){

if(ifrealistic==1){
    //set robot angle
    robot[2]=getAngle(robot,ball)-dtor(angle);
    //calibrate acellerometer
    //IMU_angle=IMU_Read+IMU_ref;
    //IMU_angle=robot[2]=getAngle(robot,ball)-dtor(angle);
    //IMU_Read+IMU_ref= getAngle(robot,ball)-dtor(angle);
    
    IMU_ref = getAngle(robot,ball)-dtor(angle) - IMU_Read;
    
    //IMU_angle=IMU_Read+IMU_ref;


    //refaccel=robot[2];

    //reset time since last got good camera data
    //refff=millis();

}
else{
    IMU_angle=IMU_Read+IMU_ref;
    robot[2]= IMU_angle;
}

}

int ifrealistic(){
  int realistic=0;
  double dx2=0;
  double dy2=0;
  double cameradist = 0;

    // check if camera and radio say ball is the same dist from the robot
  dx2= pow(robot[0]-ball[0],2);
  dy2= pow(robot[1]-ball[1],2);
  cameradist = pow(dx2+dy2,0.5);
  if(cameradist < distance+errorfilter || cameradist < distance+errorfilter || distance <5){
      return 1;
  }
  else{
      return 0;
  }
}


