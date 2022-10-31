//#include <fstream>
//#include <iostream>
//#include <vector>
//#include <string>
//#include <algorithm>
//#include <stdio.h> /* printf */
//#include <math.h>

//#include <Variables.cpp>
//#include <Functions.cpp>

//using namespace std;

//Set destination
// for now set destination == ball and theta to be normal to robot ball delta (maybe average theta with history when close later)


// go to ball ---------------------------------------------------------------------------------------------------------
// set up to push towards goal
// draw robot radius circle around ball place robot at that loction in a line with the goal
//

//Psudo code
// draw a line between ball and goal
// line composed of court angle and ball location
// go to ball
// robot y = ball y - ball radius - robot max radius
// got to ball (x) use theta and trig to make a triangle to find x and goal to ball theta
// if on wrong side of ball relative to goal need intermadiate vector to get to behind ball cant just be back up or may self goal

int main()
{

    cout << "goal x y: " << endl;
    cin >> goal[0];
    cin >> goal[1];
    cout << "Ball x y: " << endl;
    cin >> ball[0];
    cin >> ball[1];

    //ball angle inverse tan of (goalx - ball x / goal y - ball y)

    BallPath[0] = getAngle(ball, goal);
    // go to ball ---------------------------------------------------------------------------------------------------
    mode = 1;
    if (mode == 1)
    {
        // if goal above
        if (goal[1] > 5)
        {
            //find x
            // x= ball + dx    dx=-sin(theta)*radius
            destination[0]=ball[0]-(sin(BallPath[0])*radius);
            //find y
            // y= ball + dy    dy=-cos(theta)*radius
            destination[1]=ball[1]-(cos(BallPath[0])*radius);
            //find in line theta
            destination[2] = BallPath[0];
        }
        // if goal below
        else
        {
            //find x
            // x= ball + dx    dx=+sin(theta)*radius (note the+ cos()...)
            destination[0]=ball[0]+(sin(BallPath[0])*radius);
            //find y
            // y= ball + dy    dy=+cos(theta)*radius (note the+ cos()...)
            destination[1]=ball[1]+(cos(BallPath[0])*radius);
            //find in line theta
            destination[2] = BallPath[0] + dtor(180);
        }
    }

    cout << "destination x: " << destination[0] << endl;
    cout << "destination y: " << destination[1] << endl;
    cout << "destination o: " << rtod(destination[2]) << endl;
    cout << endl;

    //else if (Mode == 2)
    mode = 2;
    if (mode == 2)
    {
        // Push ball to goal ---------------------------------------------------------------------------------------------------------
        // go to ball then activate this (go to ball minus ball radius)
// if goal above
        if (goal[1] > 5)
        {
            //find x
            // x= ball + dx    dx=-sin(theta)*radius
            destination[0]=ball[0]-(sin(BallPath[0])*radiusPush);
            //find y
            // y= ball + dy    dy=-cos(theta)*radius
            destination[1]=ball[1]-(cos(BallPath[0])*radiusPush);
            //find in line theta
            destination[2] = BallPath[0];
        }
        // if goal below
        else
        {
            //find x
            // x= ball + dx    dx=+sin(theta)*radius (note the+ cos()...)
            destination[0]=ball[0]+(sin(BallPath[0])*radiusPush);
            //find y
            // y= ball + dy    dy=+cos(theta)*radius (note the+ cos()...)
            destination[1]=ball[1]+(cos(BallPath[0])*radiusPush);
            //find in line theta
            destination[2] = BallPath[0] + dtor(180);
        }
    }

    // Push ball to goal ---------------------------------------------------------------------------------------------------------
    // go to ball then activate this (go to ball minus ball radius)

    cout << "destination x: " << destination[0] << endl;
    cout << "destination y: " << destination[1] << endl;
    cout << "destination o: " << rtod(destination[2]) << endl;
}
