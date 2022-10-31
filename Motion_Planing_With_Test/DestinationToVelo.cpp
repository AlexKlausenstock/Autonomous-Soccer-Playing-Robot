//DestinationToVelo----------------------------------------------------------------------------------------------------

//Outputs Velo to move towards cordinates
//#include <VeloToMotor.cpp>
//#include <Variables.cpp>

void DestinationToVelo()
{

    // Go towards x y theta
    // robot x y theta (0 = pointing +y) clock wise is +

    // find dx and dy
    for (int i = 0; i < 2; i++)
    {
        dtranslation[i] = destination[i] - robot[i];
        //cout << "dtranslation: " << dtranslation[i] << endl;
    }
    // find dtheta
    dtranslation[2] = destination[2] - robot[2];
    //cout << "dtranslation: " << dtranslation[2] << endl;

    // convert to x and y to x,y prime (rotated corrdinate system)
    //NOTE (-) is added to to bot angle relative to court because the sourced equations use a counter clock wise is (+) sense, I use clock wise is (+) sense
    //https://en.wikipedia.org/wiki/Rotation_of_axes

    //x
    velo[0] = ((dtranslation[0] * cos(dtor(-robot[2]))) + (dtranslation[1] * sin((dtor(-robot[2])))));
    //y
    velo[1] = -(dtranslation[0] * sin(dtor(-robot[2]))) + (dtranslation[1] * cos(dtor(-robot[2])));
    //theta
    velo[2] = dtranslation[2] / 4; // /4 is to  reduce scaling of degrees vs translation units of probably inches or cm

    for(int i =0; i<3;i++){
    }
}
