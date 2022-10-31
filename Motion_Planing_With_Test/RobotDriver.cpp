#include "Variables.cpp"
#include "Functions.cpp"

#include "GetMode.cpp"
#include "ModeToDestination.cpp"
#include "DestinationToVelo.cpp"
#include "VeloToMotor.cpp"
//#include "SetGoals.cpp"
//#include "SetRobotBallEnemy.cpp"
//#include "getRobotAngle.cpp"

int main()
{
    // Call the functions to go from court to generated motor values
    //set random court values and print out results to  chekc funciton lay out works
    ball[0] = 100;
    ball[1] = 100;

    robot[0] = 50;
    robot[1] = 50;
    robot[2] = 0;

    SetGoals(0); // set goals low y val
    
    GetMode(); //take in court and set destination
    cout << "destination: " << destination[0] << endl;
    cout << "destination: " << destination[1] << endl;
    cout << "destination: " << destination[2] << endl;

    DestinationToVelo(); // get velo
    cout << "Velo: " << velo[0] << endl;
    cout << "Velo: " << velo[1] << endl;
    cout << "Velo: " << rtod(velo[2]) << endl;

    //VeloToMotor();
    cout << "Motor: " << motor[0] << endl;
    cout << "Motor: " << motor[1] << endl;
    cout << "Motor: " << motor[2] << endl;
    cout << "Motor: " << motor[3] << endl;

    // Create an output filestream object
    std::ofstream myFile("foo.csv");

    //simulte robot moving
    for (int i = 0; i <= 20; i++)
    {

        // Send data to the stream
        //myFile << "Foo\n";
        myFile << to_string(robot[0]);
        myFile << " , ";
        //myFile << to_string(robot[1]);
        //myFile << " , ";
        //myFile << to_string(rtod(robot[2]));
        //myFile << "\n";

        SetGoals(0);
        GetMode();
        PushToGoal();
        DestinationToVelo();
        VeloToMotor();
        cout<<"errorCode: " <<errorCode << endl;
        cout << endl;
        cout << "Robot x: " << robot[0] << endl;
        cout << "Robot y: " << robot[1] << endl;
        cout << "Robot o: " << rtod(robot[2]) << endl;

        cout << "destination: " << destination[0] << endl;
        cout << "destination: " << destination[1] << endl;
        cout << "destination: " << destination[2] << endl;

        cout << "Velo: " << velo[0] << endl;
        cout << "Velo: " << velo[1] << endl;
        cout << "Velo: " << rtod(velo[2]) << endl;

        cout << "Motor: " << motor[0] << endl;
        cout << "Motor: " << motor[1] << endl;
        cout << "Motor: " << motor[2] << endl;
        cout << "Motor: " << motor[3] << endl;

        if (velo[0] > 0)
        {
            robot[0] = robot[0] + 1;
        }
        if (velo[1] > 0)
        {
            robot[1] = robot[1] + 1;
        }
        if (velo[2] > 0)
        {
            robot[2] = robot[2] + 1;
        }

        if (velo[0] < 0)
        {
            robot[0] = robot[0] - 1;
        }
        if (velo[1] < 0)
        {
            robot[1] = robot[1] - 1;
        }
        if (velo[2] < 0)
        {
            robot[2] = robot[2] - 1;
        }
    }

    // Close the file
    myFile.close();
}