//Alex Klausenstock
//Determine court Mode to select a SetDestination Function

// Get the mode and set destinations
void GetMode(){
//PushToGoal();
//cout<< "Test delta"<<abs(robot[0]-destination[0])<<endl;
//cout<< "Test delta"<<abs(robot[1]-destination[1])<<endl;
//cout<< "Test delta"<<abs(robot[2]-destination[2])<<endl;
//cout<< "Test omega"<<checkBlock(robot,enemy,ball,robotRadius)<<endl;

// if robot is close push the ball
//int interval=2000; // time in millis since last camera aquisition before search for ball
if(refff+interval>millis()){
	destination[0]=robot[0];
	destination[1]=robot[1];
	destination[2]=robot[2]+1;
	//maxspeed=somethignless than 255
}
else if((abs(robot[0]-destination[0])< modeError[0]) && (abs(robot[1]-destination[1])< modeError[0]) && (abs(robot[2]-destination[2])< rtod(modeError[2])) ){
    //push ball
    //mode=1;
    PushToGoal();
}
// if robot not close to ball and ball not blocked go to ball
else if((abs(robot[0]-destination[0])> modeError[0]) || (abs(robot[1]-destination[1])> modeError[0]) || (abs(robot[2]-destination[2])> rtod(modeError[2])) && (checkBlock(robot,enemy,ball,robotRadius) == 0) ){
    GoToBall();
}
// if robot not close to ball and ball blocked go to intermediate // fan the ball
else if((abs(robot[0]-destination[0])> modeError[0]) || (abs(robot[1]-destination[1])> modeError[0]) || (abs(robot[2]-destination[2])> rtod(modeError[2])) && (checkBlock(robot,enemy,ball,robotRadius) == 1)){
    GoToBallIntermediate();
}
else{
    errorCode=10;
}
//
//else if(){

//}

}