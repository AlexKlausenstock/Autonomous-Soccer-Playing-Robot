//VeloToMoror-----------------------------------------------------------------------------------------------------
void VeloToMotor()
{

    // Normalization stuff for driver code

    // velo [0] (front and rear wheels towards and away from eachother) // +x direction
    // velo[1] (all in 1 direction) // +y dir forward*
    // velo[2] (left weels + right wheels -) // rotation clockwise
    // motor control motors[4] Index: (clockwise order, front left first) 
    //Index diagram:
    //[0]---[1]
    //| robot |
    //[4]___[3]

    //1 get desired direction in x y and rotation and convert to motor values via supper position
    //2 normalize to limit max motor vectors to 255
    //3 Order motors to move

    // eliminate very small motion to creat motion dead zone (because it will normalize to max 255)
    /*
    for(int i=0; i<3;i++){
        if (velo[i]<.1){
            velo[i]=0;
        }
    }
    */

    // find motor values
    //front left
    motor[0] = velo[1] + velo[0] + velo[2];
    //front right
    motor[1] = velo[1] - velo[0] - velo[2];
    //back right
    motor[2] = velo[1] + velo[0] - velo[2];
    //back left
    motor[3] = velo[1] - velo[0] + velo[2];

    // normalize by the max motor value
    // normm = max element of motor
    norm=0;
    for (int i = 0; i <= 3; i++)
    {
        if (abs(motor[i]) > norm)
        {
            norm = abs(motor[i]);
        }
    }
    // Scale / map largest motor value to be 255 only if norm !=0
    //if norm == 0 then all velo=0 and trying to hold still also dividing by zero would cause an error
    if(norm!=0)
    {
        for (int i = 0; i <= 3; i++)
        {
            motor[i] = motor[i] / norm;
            motor[i] = motor[i] * MaxSpeed;
            //MaxSpeed = 255 which is 100% output for pwm in arduino
            
        }
    }

    // round to integer for pwm value
    for (int i = 0; i <= 3; i++)
    {
        motor[i] = floor(motor[i]);
    }
    // call spinMotor function with these motor functions in arduino
}
