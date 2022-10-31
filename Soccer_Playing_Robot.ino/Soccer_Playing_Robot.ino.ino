// READ ME!!!
// When Puting code together check each ********** flag
// Go down the list and check each section has code or a "//no code" comment
// Finally, check no conflicting variables
// Assume only code is VS code, MotorToSpin, IMU, LaserRangeFinder, Fan, PieToArduino


//VS Code Variables************************************************
// do not copy the includes or using name space std into arduino
// VeloToMotor
//DestiantionToVelo
//GoTo

// VelotoMotor
double velo[3] = {0, 0, 0};     // y x theta forward+ right+ clockwise+
double motor[4] = {0, 0, 0, 0}; // motor pwm signals
double norm = 0;                // for calculating max of motor to make sure max(motor)<=255

double robot[3] = {0, 0, 0}; // robot coordinates x y theta (degrees clockwise) datum at +y axis
double enemy[2] = {0, 0};    // enemy court position
double ball[2] = {0, 0};     // ball court posoition
double ballVelo[2] = {0, 0}; // ball velocity
int MaxSpeed = 255;          // normalizing value for max speed

double destination[3] = {0, 0, 0};  // assigned destination
double dtranslation[3] = {0, 0, 0}; // difference between robot and destiantion for velo calc

double goal[2] = {0, 0};     // goal location
double goal2[2] = {0, 0};    // Enemy goal location
double BallPath[2] = {0, 0}; // angle 1 then 2 of planned ball path stored in radians because it's not needed in degrees

double ballRadius = 27;                            // ball radius built in buffer
double ballRadiusPush = 18;                        // ball radius minus some
double robotRadius = 42;                           // robot max radius 14 inches
double radiusBallRobot = ballRadius + robotRadius; // radius to set up by the ball for destination calc
double radiusPush = ballRadiusPush + robotRadius;  // radius to push the ball for destination calc

int errorCode = 0; // error code to be used for durring use debug 1= getAngle broke 2

//SetMode
int mode = 0;                       //Control movement mode
double modeError[3] = {2, 2, 5};    // error of robot
double modeGoToBall[3] = {1, 1, 1}; // error of robot to set up on ball destination

//Check Court Conditions Variables

//Get Destination variables

int fan = 0; // Fan Speed 0 to 255

//Filter Camera Data
double angle = 0; // angle from pixle to angle .ino
int errorfilter = 10;
double distance=0; // Distance from radius from camera

//Set IMU angle
double IMU_Read = 0; // value read from IMU
double IMU_ref = 0; // Reference value used to calibrate IMU based on camera data
double IMU_angle = 0; // calibrated robot[2] angle relative to the court

//SetRobotBallEnemy.cpp vars
int robotColor = 0;
int ballColor = 0;
int enemyColor = 0;
//Get Mode
int refff; // ref for time since last got realistic data from camera

int interval = 10000; // time in millis since last camera aquisition before search for ball

//motorToSpin Variables************************************************

//MotorSpin Variables
//PIN VARIABLES
//the motor direction is controled by AIN or BIN 1 & 2
//Driver 1
// motor 1 front left
const int A1IN1 = 22;           //control pin 1
const int A1IN2 = 23;           //control pin 2
const int PWM1A = 3;            //speed control pin on the motor driver
// motor 2 front right
const int B2IN1 = 24;           //control pin 1
const int B2IN2 = 25;           //control pin 2
const int PWM2B = 4;            //speed control pin on the motor driver

// Driver 2
// motor 3 back right
const int A3IN1 = 26;           //control pin 1
const int A3IN2 = 27;           //control pin 2
const int PWM3A = 5;            //speed control pin on the motor driver
// motor 4 back left
const int B4IN1 = 28;          //control pin 1
const int B4IN2 = 29;          //control pin 2
const int PWM4B = 6;           //speed control pin on the motor driver



//IMU Variables************************************************
//Alex Note: Error increase = f(time, d angle) about  0.5 deg/ sec error (fickle) and maybe +5 -+15 deg per 360
//also 90 deg not == 90  90 deg == 45
//So convert to 2*angle_pitch then convert to rad
//mpuAngle=dtor(angle_pitch*2);
//double IMU_Read;

//Include LCD and I2C library for debug
//#include <LiquidCrystal_I2C.h>
#include <Wire.h>

//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

//Initialize the LCD library
//LiquidCrystal_I2C lcd(0x27,16,2);

//LaserRangeFinder Variables************************************************
//Done

//Fan Variables************************************************
#include <Servo.h>
Servo ESC;
//int fan;


//PieToArduino Variables************************************************
String nom = "Arduino";
String msg;
String x_coord;
String rad;
int xcoord=0;
int radius=0;
int Reference = 0;

//need to add a time var to remember when the last time a sample was taken to determine how to take data
int lastSample=0;


//Radio Code Variables************************************************
#include <SPI.h>
#include <RFM69.h>

// Addresses for this node. CHANGE THESE FOR EACH NODE!
#define NETWORKID     10  // Must be the same for all nodes
#define MYNODEID      2   // My node ID
#define TONODEID      1   // Origin node ID

// RFM69 frequency, uncomment the frequency of your module:
//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ

// AES encryption (or not):
#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):
#define USEACK        false // Request ACKs or not

// Create a library object for our RFM69HCW module:
RFM69 radio;

#define PACKSIZE 25 // Maximum Packetsize is 25,if 4 digit for x and y, and static characters
#define GrnID 1 // #defined constants for clearer code
#define BluID 2
#define YelID 3

int gx, gy, bx, by, yx, yy; //Global variables for storing x and y coordinates of green, blue and yellow ball locations
char buildInt[4];


//Pixle to Angle Variables************************************************
//int distance = 0;
//int radius = 0;
//int xcoord=0;
//double angle=0;


//SetGoal Variables************************************************
// Include VS Code
// So goal top = 275 , 12.5
// Goal bottom = 270, 360
// set goals for set up
// if input 0 set goal to low y value ("top")
//double goal[2] = {0, 0};     // goal location
//double goal2[2] = {0, 0};    // Enemy goal location


//SetRobotBallEnemy Variables************************************************
//our robot is 
//1==green
//2==blue
//yellow (no number)

//int gx, gy, bx, by, yx, yy;// Global variables for storing x and y coordinates of green, blue and yellow ball locations
//int errorCode=0;
//double robot [2]= {0,0};
//double enemy [2]= {0,0};
//double ball [2]= {0,0};

//int robotColor=0;
//int enemyColor=0;


//getRobotAngle Variables************************************************

void setup() {

  
//Debug************************************************
Serial3.begin(9600);
pinMode(30,OUTPUT);

  //motorToSpin Setup************************************************
  pinMode(A1IN1, OUTPUT);
  pinMode(A1IN2, OUTPUT);
  pinMode(PWM1A, OUTPUT);
  // front right
  pinMode(B2IN1, OUTPUT);
  pinMode(B2IN2, OUTPUT);
  pinMode(PWM2B, OUTPUT);
  // back right
  pinMode(A3IN1, OUTPUT);
  pinMode(A3IN2, OUTPUT);
  pinMode(PWM3A, OUTPUT);
  // back left
  pinMode(B4IN1, OUTPUT);
  pinMode(B4IN2, OUTPUT);
  pinMode(PWM4B, OUTPUT);
  
  //IMU Setup************************************************
  //Serial.begin(9600);
  Wire.begin();                                                        //Start I2C as master
  //Serial.begin(57600);                                               //Use only for debugging
  //pinMode(13, OUTPUT);                                                 //Set output 13 (LED) as output
  
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  //digitalWrite(13, HIGH);                                              //Set digital output 13 high to indicate startup

  //lcd.begin();                                                         //Initialize the LCD
  //lcd.backlight();                                                     //Activate backlight
  //lcd.clear();                                                         //Clear the LCD

  //lcd.setCursor(0,0);                                                  //Set the LCD cursor to position to position 0,0
  //lcd.print("  MPU-6050 IMU");                                         //Print text to screen
  //lcd.setCursor(0,1);                                                  //Set the LCD cursor to position to position 0,1
  //lcd.print("     V1.0");                                              //Print text to screen

  //delay(1500);                                                         //Delay 1.5 second to display the text
  //lcd.clear();                                                         //Clear the LCD
  
  //lcd.setCursor(0,0);                                                  //Set the LCD cursor to position to position 0,0
  //lcd.print("Calibrating gyro");                                       //Print text to screen
  //lcd.setCursor(0,1);                                                  //Set the LCD cursor to position to position 0,1
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    //if(cal_int % 125 == 0)lcd.print(".");                              //Print a dot on the LCD every 125 readings
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset

  //lcd.clear();                                                         //Clear the LCD
  
  //lcd.setCursor(0,0);                                                  //Set the LCD cursor to position to position 0,0
  //lcd.print("Pitch:");                                                 //Print text to screen
  //lcd.setCursor(0,1);                                                  //Set the LCD cursor to position to position 0,1
  //lcd.print("Roll :");                                                 //Print text to screen
  
  //digitalWrite(13, LOW);                                               //All done, turn the LED off
  
  loop_timer = micros();     
  
  //LaserRangeFinder Setup************************************************
  //Done
  //Fan Setup************************************************
  
  //Serial.begin(9600);
  fan=0;
  ESC.attach(13,1000,2000);

  while(millis()<3000){
    fan = 255;
    ESC.write(fan);
  }
  while (millis()<6000 && millis()>3000){
    fan = 0;
    ESC.write(fan);
  }
  //PieToArduino Setup************************************************
  Serial.begin(9600);
  
  //Radio Code Setup************************************************
  
  // Open a serial port so we can see the data we receive
  //Serial.begin(38400);
  //Serial.print("Node ");
  //Serial.print(MYNODEID,DEC);
  //Serial.println(" ready");  

  sprintf(buildInt, "    "); // Clear the buildInt array

  // Initialize the RFM69HCW:
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW

  // Turn on encryption if desired:
  if (ENCRYPT)
    radio.encrypt(ENCRYPTKEY);
    
  //Pixle to Angle Setup************************************************
  //Done
  
  //SetGoal Setup************************************************
  pinMode(32,INPUT);
  
  //SetRobotBallEnemy Setup************************************************
  pinMode(33,INPUT);

  //getRobotAngle Variables************************************************
  
}

void loop() {
//Debug************************************************
//Serial3.begin(9600);
//pinMode(30,OUTPUT);
//digitalWrite(30,HIGH);
  
  //Do Stuff Loop************************************************

  //call spin functions Done
  //Set Goals Called
  //Set Ball Color Called
  //


  setRobot2();
  
  GetMode();
  DestinationToVelo();
  VeloToMotor();

  //debug

  //Serial3.println("refff: ");
  //Serial3.println(refff);
  Serial3.println(xcoord);
  
  
  
  









































  
  //motorToSpin Loop************************************************
  spinMotor1(motor[0]);
  spinMotor2(motor[1]);
  spinMotor3(motor[2]);
  spinMotor4(motor[3]);
  
  //IMU Loop************************************************
  

  

  read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  
  //write_LCD(); 
  
  //Write the roll and pitch values to the LCD display

  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer

  //Serial.println("Pitch:");
  //Serial.println(angle_pitch);
  //Serial.println(angle_roll);
  //mpuAngle=dtor(angle_pitch*2);
  //IMU_Read=dtor(angle_pitch*2);
  
  IMU_Read=dtor(angle_roll*2);
  //Serial.println(mpuAngle);
  
  //LaserRangeFinder Loop************************************************
  //done
  
  //Fan Loop************************************************
  fanSet(fan);
  
  //PieToArduino Loop************************************************
  
  readSerialPort();

  if (msg != "") {
    sendData();
    Reference = millis();
  }
  //delay(500);
  
  //Radio Code Loop************************************************
  // RECEIVING
  // In this section, we'll check with the RFM69HCW to see
  // if it has received any packets:
  int j=0;
  char data[PACKSIZE];
  for (j=0; j<PACKSIZE; j++) data[j]=0; //Clear the data array to make sure there is no left over junk in it
  if (radio.receiveDone()) { // Got one!
    // Print out the information:
    //Serial.print("received from node ");
    //Serial.print(radio.SENDERID, DEC);
    //Serial.print(", message [");

    // The actual message is contained in the DATA array,
    // and is DATALEN bytes in size:
    for (byte i = 0; i < radio.DATALEN; i++){
      Serial.print((char)radio.DATA[i]);
      data[i]=(char)radio.DATA[i];
      j=i;
      }
    // RSSI is the "Receive Signal Strength Indicator",
    // smaller numbers mean higher power.
    //Serial.print("], RSSI ");
    //Serial.println(radio.RSSI);
    
    // Now parse the data received into coordinates as integers
    j=0;
    while (j<PACKSIZE) {  //Work through the data and look for color letter indicators
      if(data[j]=='G') {  //Found green
        //Serial.print("Green:");
        pullCoords(data,j,GrnID); //Get the green x and y (as ints) and print them.
        //Serial.print(gx);
        //Serial.print(" ");
        //Serial.println(gy);
        break;                    // Assuming green is only color in its packet
      } else if (data[j]=='B') { //Found blue
        //Serial.print("Blue:");
        pullCoords(data,j,BluID); //Get the blue x and y (as ints) and print them.
        //Serial.print(bx);
        //Serial.print(" ");
        //Serial.println(by);
        j++;                      //Asuming blue and yellow and together, blue first
      } else if (data[j]=='Y') { //Found yellow
        //Serial.print("Yellow:");
        pullCoords(data,j,YelID); //Get the yellow x and y (as ints) and print them.
        //Serial.print(yx);
        //Serial.print(" ");
        //Serial.println(yy);
        break;                    // Assuming yellow is at the end of its packet      
      } else {
        j++;                      // Go to next character
      }
    }
  }
  
  //Pixle to Angle Loop************************************************
  angle=pixeltoangle(radius,xcoord);
  
  //SetGoal Loop************************************************
  if (digitalRead(32)==HIGH){
    SetGoals(1);
  }
  else{
    SetGoals(0);
  }
  
  //SetRobotBallEnemy Loop************************************************
  if(digitalRead(33)==HIGH){
    SetRobotBallEnemyColor(1,2);
    getRobotBallEnemy();
  }
  else{
    SetRobotBallEnemyColor(2,1);
    getRobotBallEnemy();
  }
  
}

  //VS Code Functions (IN ORDER!!)************************************************
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
        refTime=millis();
        for(int i=0; i<errorCode; i++){
            while(refTime<refTime+1000){
                digitalWrite(30,HIGH);
            }
            //refTime=millis();
            while(refTime<refTime+1000){
                digitalWrite(30,LOW);
            }
        }
        return 1;
    }
}

// Get the mode and set destinations
void GetMode(){
//PushToGoal();
//cout<< "Test delta"<<abs(robot[0]-destination[0])<<endl;
//cout<< "Test delta"<<abs(robot[1]-destination[1])<<endl;
//cout<< "Test delta"<<abs(robot[2]-destination[2])<<endl;
//cout<< "Test omega"<<checkBlock(robot,enemy,ball,robotRadius)<<endl;

// if robot is close push the ball
int interval=2000; // time in millis since last camera aquisition before search for ball
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

//Filter Incorrect Radio function
//use robot cordinates xy and ball xy, compare to robot vs ball angle and dist to see if realistic

//get angle from camera
//double angle = 0;

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
robot[2]= IMU_angle=IMU_Read+IMU_ref;
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





  //motorToSpin Function************************************************
  void spinMotor1(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(A1IN1, HIGH);                         //set pin 1 to high
    digitalWrite(A1IN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(A1IN1, LOW);                          //set pin 1 to low
    digitalWrite(A1IN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(A1IN1, LOW);                          //set pin 1 to low
    digitalWrite(A1IN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWM1A, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void spinMotor2(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(B2IN1, HIGH);                         //set pin 1 to high
    digitalWrite(B2IN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(B2IN1, LOW);                          //set pin 1 to low
    digitalWrite(B2IN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(B2IN1, LOW);                          //set pin 1 to low
    digitalWrite(B2IN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWM2B, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void spinMotor3(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(A3IN1, HIGH);                         //set pin 1 to high
    digitalWrite(A3IN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(A3IN1, LOW);                          //set pin 1 to low
    digitalWrite(A3IN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(A3IN1, LOW);                          //set pin 1 to low
    digitalWrite(A3IN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWM3A, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void spinMotor4(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(B4IN1, HIGH);                         //set pin 1 to high
    digitalWrite(B4IN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(B4IN1, LOW);                          //set pin 1 to low
    digitalWrite(B4IN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(B4IN1, LOW);                          //set pin 1 to low
    digitalWrite(B4IN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWM4B, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}
  
  //IMU Function************************************************
  

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

}
/*
void write_LCD(){                                                      //Subroutine for writing the LCD
  //To get a 250Hz program loop (4us) it's only possible to write one character per loop
  //Writing multiple characters is taking to much time
  if(lcd_loop_counter == 14)lcd_loop_counter = 0;                      //Reset the counter after 14 characters
  lcd_loop_counter ++;                                                 //Increase the counter
  if(lcd_loop_counter == 1){
    angle_pitch_buffer = angle_pitch_output * 10;                      //Buffer the pitch angle because it will change
    lcd.setCursor(6,0);                                                //Set the LCD cursor to position to position 0,0
  }
  if(lcd_loop_counter == 2){
    if(angle_pitch_buffer < 0)lcd.print("-");                          //Print - if value is negative
    else lcd.print("+");                                               //Print + if value is negative
  }
  if(lcd_loop_counter == 3)lcd.print(abs(angle_pitch_buffer)/1000);    //Print first number
  if(lcd_loop_counter == 4)lcd.print((abs(angle_pitch_buffer)/100)%10);//Print second number
  if(lcd_loop_counter == 5)lcd.print((abs(angle_pitch_buffer)/10)%10); //Print third number
  if(lcd_loop_counter == 6)lcd.print(".");                             //Print decimal point
  if(lcd_loop_counter == 7)lcd.print(abs(angle_pitch_buffer)%10);      //Print decimal number

  if(lcd_loop_counter == 8){
    angle_roll_buffer = angle_roll_output * 10;
    lcd.setCursor(6,1);
  }
  if(lcd_loop_counter == 9){
    if(angle_roll_buffer < 0)lcd.print("-");                           //Print - if value is negative
    else lcd.print("+");                                               //Print + if value is negative
  }
  if(lcd_loop_counter == 10)lcd.print(abs(angle_roll_buffer)/1000);    //Print first number
  if(lcd_loop_counter == 11)lcd.print((abs(angle_roll_buffer)/100)%10);//Print second number
  if(lcd_loop_counter == 12)lcd.print((abs(angle_roll_buffer)/10)%10); //Print third number
  if(lcd_loop_counter == 13)lcd.print(".");                            //Print decimal point
  if(lcd_loop_counter == 14)lcd.print(abs(angle_roll_buffer)%10);      //Print decimal number
}
*/

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

  //LaserRangeFinder Function************************************************
  //Done
  
  //Fan Function************************************************
void fanSet(int fan){
ESC.write(fan);
}

  //PieToArduino Function************************************************
  
void readSerialPort() {
  msg = "";
  x_coord = "";
  if (Serial.available()) {
    //lastSample is for external use
    lastSample=millis();
    delay(5);
    while (Serial.available() > 0) {
      msg += (char)Serial.read();
    }
    //Serial.print(msg.substring(0,3));
    if (msg.substring(0,4) == "CAMX") {
      x_coord = msg.substring(5);
    }
    if (msg.substring(0,4) == "CAMr") {
      rad = msg.substring(5);
    }
    Serial.flush();
    //convert data to int type
    xcoord=x_coord.toInt();
    radius=rad.toInt();
    
  }
}

void sendData() {
  //write data
  Serial.print(nom);
  Serial.print(" received : ");
  Serial.print(msg);
  if (x_coord != "") {
    Serial.print(" X-Coordinate: ");
    Serial.print(x_coord);
  }
}

  //Radio Code Function************************************************
  
void pullCoords(char packet[], int letterLoc, int color) {
    int BrS,Comma,BrE = 0; // Array locations of bracket start character, comma and bracket end character.
    int i=0;
  
    BrS=letterLoc+1; // The start bracket will be immediately after the color letter indicator
    for (i = BrS+1; i < PACKSIZE; i++) {
      if(packet[i]==',') Comma=i;  // Find the location of the comma that divides x and y coordinates
      if(packet[i]=='>') {
        BrE=i;  // Find the location of the end bracket
        break;  // And get out of for loop
      }
    }
    sprintf(buildInt, "    ");  // Clear the buildInt array      
    for (i = 0; i < (Comma-(BrS+1)); i++) {  // Work through the chars that represent the x coordinate
      buildInt[i]=packet[(BrS+1+i)];         // and copy them into the buildInt array
    }
    buildInt[(Comma-(BrS+1))]='\0';  // Terminate the buildInt array to make it easy on the converter
    switch (color) {                 // Convert the buildInt array to an integer and store in the correspoding color x coordinate
      case GrnID: 
        gx=atoi(buildInt); // Green x coord
        break;
      case BluID: 
        bx=atoi(buildInt); // Blue x coord
        break;
      case YelID: 
        yx=atoi(buildInt); // Yellow x coord
        break;
    }
    sprintf(buildInt, "    ");  // Clear the buildInt array 
    for (i = 0; i < (BrE-(Comma+1)); i++) {  // Work through the chars that represent the y coordinate
      buildInt[i]=packet[(Comma+1+i)];         // and copy them into the buildInt array
    }
    buildInt[(BrE-(Comma+1))]='\0';  // Terminate the buildInt array to make it easy on the converter
    switch (color) {                 // Convert the buildInt array to an integer and store in the correspoding color y coordinate
      case GrnID: 
        gy=atoi(buildInt); // Green y coord
        break;
      case BluID: 
        by=atoi(buildInt); // Blue y coord
        break;
      case YelID: 
        yy=atoi(buildInt); // Yellow y coord
        break;
    }
}

  
  //Pixle to Angle Function************************************************
  
  double pixeltoangle(int radius,int  xpixel) {
  distance = -1.52*radius + 137;
  int xpixeladjusted = xpixel - 320;
  float relativePixel = xpixeladjusted/320;
  double Angle = relativePixel*32.6;
  return Angle;
}

  //SetGoal Function************************************************
  
void SetGoals(int top){
  //set Goal to low y value
if (top==0){
    goal[0]= 275;
    goal[1]= 12.5;
    goal2[0]= 270;
    goal2[1]= 360;
}
//set Goal to high y value
else{
    goal2[0]= 275;
    goal2[1]= 12.5;
    goal[0]= 270;
    goal[1]= 360;
}
}


  
  //SetRobotBallEnemy Function************************************************
  
void SetRobotBallEnemyColor(int robot, int enemy){
    if(robot==1){
        robotColor=1;
    }
    else if(robot ==2){
        robotColor=2;
    }
    else{
        errorCode=2;
    }

    if(enemy==1){
        enemyColor=1;
    }
    else if (enemy==2){
        enemyColor=2;
    }
    else{
        errorCode=2;
    }
}

void getRobotBallEnemy(){
        if(robotColor==1){
        robot[0]=gx;
        robot[1]=gy;
    }
    else if(robotColor ==2){
        robot[0]=bx;
        robot[1]=by;
    }
    else{
        errorCode=2;
    }

    ball[0]=yx;
    ball[1]=yy;

    if(enemyColor==1){
        enemy[0]=gx;
        enemy[1]=gy;
    }
    else if (enemyColor==2){
        enemy[0]=bx;
        enemy[1]=by;
    }
    else{
        errorCode=2;
    }
}
