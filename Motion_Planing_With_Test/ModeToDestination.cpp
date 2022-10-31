//ModeToDestinationFunctions----------------------------------------------------------------------------------------------------------
//Push the ball
// Sets destination of robot[] uses ball[] goal[]
void PushToGoal()
{
    // Push ball to goal ---------------------------------------------------------------------------------------------------------
    // go to ball then activate this (go to ball minus ball radius)
    // if goal above
    BallPath[0] = getAngle(ball, goal);
    if (goal[1] > 5)
    {
        //find x
        // x= ball + dx    dx=-sin(theta)*radius
        destination[0] = ball[0] - (sin(BallPath[0]) * radiusPush);
        //find y
        // y= ball + dy    dy=-cos(theta)*radius
        destination[1] = ball[1] - (cos(BallPath[0]) * radiusPush);
        //find in line theta
        destination[2] = BallPath[0];
    }
    // if goal below
    else
    {
        //find x
        // x= ball + dx    dx=+sin(theta)*radius (note the+ cos()...)
        destination[0] = ball[0] + (sin(BallPath[0]) * radiusPush);
        //find y
        // y= ball + dy    dy=+cos(theta)*radius (note the+ cos()...)
        destination[1] = ball[1] + (cos(BallPath[0]) * radiusPush);
        //find in line theta
        destination[2] = BallPath[0] + dtor(180);
    }
}

//Set up behind ball looking at goal
// Sets destination of robot[] uses ball[] goal[]
void GoToBall()
{
    // go to ball ---------------------------------------------------------------------------------------------------
    //ball angle inverse tan of (goalx - ball x / goal y - ball y)
    BallPath[0] = getAngle(ball, goal);
    // if goal above
    if (goal[1] > goal2[1])
    {
        //find x
        // x= ball + dx    dx=-sin(theta)*radius
        destination[0] = ball[0] - (sin(BallPath[0]) * radiusBallRobot);
        //find y
        // y= ball + dy    dy=-cos(theta)*radius
        destination[1] = ball[1] - (cos(BallPath[0]) * radiusBallRobot);
        //find in line theta
        destination[2] = BallPath[0];
    }
    // if goal below
    else
    {
        //find x
        // x= ball + dx    dx=+sin(theta)*radius (note the+ cos()...)
        destination[0] = ball[0] + (sin(BallPath[0]) * radiusBallRobot);
        //find y
        // y= ball + dy    dy=+cos(theta)*radius (note the+ cos()...)
        destination[1] = ball[1] + (cos(BallPath[0]) * radiusBallRobot);
        //find in line theta
        destination[2] = BallPath[0] + dtor(180);
    }
}

//go to behind first facing second at dist from center away from first center
void GoToFacing(double first[3],double second[3],double DistFromCenter){

    // go to object ---------------------------------------------------------------------------------------------------
    //ball angle inverse tan of (goalx - ball x / goal y - ball y)
    BallPath[1] = getAngle(first, second);
    // if goal above
    if (goal[1] > goal2[1])
    {
        //find x
        // x= ball + dx    dx=-sin(theta)*radius
        destination[0] = first[0] - (sin(BallPath[1]) * DistFromCenter);
        //find y
        // y= ball + dy    dy=-cos(theta)*radius
        destination[1] = first[1] - (cos(BallPath[1]) * DistFromCenter);
        //find in line theta
        destination[2] = BallPath[1];
    }
    // if goal below
    else
    {
        //find x
        // x= ball + dx    dx=+sin(theta)*radius (note the+ cos()...)
        destination[0] = ball[0] + (sin(BallPath[0]) * radiusBallRobot);
        //find y
        // y= ball + dy    dy=+cos(theta)*radius (note the+ cos()...)
        destination[1] = ball[1] + (cos(BallPath[0]) * radiusBallRobot);
        //find in line theta
        destination[2] = BallPath[0] + dtor(180);
    }
}

void HoldStill(){
    destination[0]=robot[0];
    destination[1]=robot[1];
    destination[2]=robot[2];
}

// check if an object is blocking inbetween two points. return: 0 ==> not Blocked; 1 ==> blocked.
bool checkBlock(double start[3], double block[3], double end[3], double rBlock)
{
    //check if block is in line and large enough to block
    double theta2 = 0;
    double h2 = 0;
    theta2 = getAngle(start, end) - getAngle(start, block);
    h2 = sqrt(pow(start[0] - block[0], 2) + pow(start[1] - block[1], 2));
    // -1 to radius block to make us a little aggresive so any tiny amount of blocking will not stop us
    if (sin(dtor(theta2) * h2) < rBlock - 1)
    {
        // if dist of block to our path line < block radius ==>  on the line of start end
        //check if is between start-end
        // if end above start
        if (end[1] > start[1] && block[1] > start[1] && block[1] < end[1])
        {
            // not open
            return 1;
        }
        //if end below start
        else if (end[1] < start[1] && block[1] < start[1] && block[1] > end[1])
        {
            // not open
            return 1;
        }
        else
        {
            // enemy in line but on other side of end from start
            return 0;
        }
    }
    else
    {
        // block not even on a line between start and end
        return 0;
    }
}

//1 == on 0==off
//Must be turned off ***
void FanTheBall(int on)
{
    if (on==1){
    double range = 10;
    //set destination angle
    destination[2] = getAngle(robot, ball);
    // turn fan on if robot is roughly looking at the ball
    if (robot[2] < getAngle(robot, ball) + dtor(range) && robot[2] > getAngle(robot, ball) - dtor(range))
    {
        //turn on fan
        //un comment before running in arduino**********************************************************************************************
        //fanSet(255);
        fan=255;
    }
    }
    else if(on==0){
        //fan off
        //un comment before running in arduino**********************************************************************************************
        //fanSet(255);
        fan=0;
    }
    else{
        errorCode=1;
    }
}

//goes and puppyguard enemy untill they get out of the way, if in line with goal will fan the ball
void GoToBallIntermediate()
{
    // if enemy in the way, go to enemy
    if(checkBlock(robot,enemy,ball,robotRadius)==1){
        GoToFacing(enemy,ball,robotRadius);
        //fan the ball if ball not too close to edge or if ball blocks goal.
        if(checkBlock(robot,ball,goal,ballRadius)==1){
            FanTheBall(1);
        }
    }
    else{
        PushToGoal();
    }
}

// make genertic function to set up behind start going to end?

// make function to get to point when end is blocked

//1 blocks goal 0 doesn't block
int CheckBallBlockGoal(){
    if(checkBlock(robot,ball,goal,ballRadius) == 1){
        return 1;
        //FanTheBall(1);
    }
    else{
        return 0;
    }
}

//1 open 0 not open
int CheckBallOpen()
{
    // check if enemy robot between robot and ball
    if (checkBlock(robot, enemy, ball, robotRadius) == 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

//1 open 0 not open
int CheckGoalOpen()
{
    // check if enemy robot between ball and goal
    if (checkBlock(ball, enemy, goal, robotRadius) == 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// incomplete
void ConvertCoodrinates()
{
    // maybe make this?
    // convert x y to x' y'
    //x
    velo[0] = ((dtranslation[0] * cos(dtor(-robot[2]))) + (dtranslation[1] * sin((dtor(-robot[2])))));
    //y
    velo[1] = -(dtranslation[0] * sin(dtor(-robot[2]))) + (dtranslation[1] * cos(dtor(-robot[2])));
}



