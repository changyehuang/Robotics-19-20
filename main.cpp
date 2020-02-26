/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\alexa                                            */
/*    Created:      Tue Feb 25 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
//           
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "ButtonClass.h"

using namespace vex;

competition Competition;


double slowDrive = 0.7, slowInt;


//const int WHEEL_CIRCUMFERENCE = 33;
//33 cm

void sleep(int i){
  vex::task::sleep(i);
}

//run a motor for a set speed
void run(motor motor1, int speed){
motor1.setVelocity(speed, rpm);
motor1.spin(fwd);
}

//hold one motor
void motorBrake(motor motor1){
motor1.stop(hold);
}

//hold two motos
void motorBrake(motor motor1, motor motor2){
  motor1.stop(hold);
  motor2.stop(hold);
}

//run a pair of motors
void run(motor motor1, motor motor2, int speed)
{
  motor1.setVelocity(speed, rpm);
  motor1.spin(fwd);
  motor2.setVelocity(speed, rpm);
  motor2.spin(fwd);
}
//drive with speed of either wheel
void drive(int left, int right){
  if(left == 0 && right == 0)
  {
    motorBrake(DRIVE_LB, DRIVE_LF);
    motorBrake(DRIVE_RB, DRIVE_RF);
  }else{
  run(DRIVE_RF, right);
  run(DRIVE_LF, left);
  run(DRIVE_LB, left);
  run(DRIVE_RB, right);
}

}


void drive(int left, int right, int time){
  drive(left, right);
  sleep(time);
  drive(0, 0);
}

void sideDrive(int speed){
  run(DRIVE_RF, speed);
  run(DRIVE_LF, -speed);

  run(DRIVE_LB, speed);
  run(DRIVE_RB, -speed);
}



void pidDrive(double d, float speed1, float speed2){
  DRIVE_LB.resetRotation();
  DRIVE_RB.resetRotation();
double distance = -((d / 15) * 23);

  static float PID_INTEGRAL_LIMIT = 50;
  static int PID_DRIVE_MAX = 127;
  static int PID_DRIVE_MIN = -127;

  float  encoderValueL = 0;
  float  encoderValueR = 0;
  float  pid_Kp = 2.0;
  float  pid_Ki = 0.04;
  float  pid_Kd = 0.0;

  float  pidErrorL = 1000;
  float  pidErrorR = 1000;
  float  pidLastErrorL = 0;
  float  pidLastErrorR = 0;
  float  pidIntegralL = 0;
  float  pidIntegralR = 0;
  float  pidDerivativeL = 0;
  float  pidDerivativeR = 0;
  float  pidDriveL;
  float  pidDriveR;

  pidLastErrorL  = 0;
  pidLastErrorR  = 0;
  pidIntegralL   = 0;
  pidIntegralR   = 0;


while(fabs(pidErrorL) > 0.03 || fabs(pidErrorR) > 0.03 ){

encoderValueL = -DRIVE_LB.rotation(vex::rotationUnits (rev));
encoderValueR = -DRIVE_RB.rotation(vex::rotationUnits (rev));


pidErrorL = encoderValueL - distance;
pidErrorR = encoderValueR - distance;


if( pid_Ki != 0 )
                {
                // If we are inside controlable window then integrate the error
                if( fabs(pidErrorL) < PID_INTEGRAL_LIMIT)
                    pidIntegralL = pidIntegralL + pidErrorL;
                     else
                    pidIntegralL = 0;
                     
                if(fabs(pidErrorR) < PID_INTEGRAL_LIMIT)
                    pidIntegralR = pidIntegralR + pidErrorR;
                else
                    pidIntegralR = 0;
                

                }
            else{
                pidIntegralL = 0;
                pidIntegralR = 0;
            }

            // calculate the derivative
            pidDerivativeL = pidErrorL - pidLastErrorL;
            pidLastErrorL  = pidErrorL;
            pidDerivativeR = pidErrorR - pidLastErrorR;
            pidLastErrorR  = pidErrorR;


            // calculate drive
            pidDriveL = (pid_Kp * pidErrorL) + (pid_Ki * pidIntegralL) + (pid_Kd * pidDerivativeL);
            pidDriveR = (pid_Kp * pidErrorR) + (pid_Ki * pidIntegralR) + (pid_Kd * pidDerivativeR);

            // limit drive
            if( pidDriveL > PID_DRIVE_MAX )
                pidDriveL = PID_DRIVE_MAX;
            if( pidDriveL < PID_DRIVE_MIN )
                pidDriveL = PID_DRIVE_MIN;

            if( pidDriveR > PID_DRIVE_MAX )
                pidDriveR = PID_DRIVE_MAX;
            if( pidDriveR < PID_DRIVE_MIN )
                pidDriveR = PID_DRIVE_MIN;
            


            // send to motor
            drive(pidDriveL * speed1, pidDriveR * speed2);
            }

}

void pidDrive(double dL, double dR, float speed1, float speed2){
DRIVE_LB.resetRotation();
DRIVE_RB.resetRotation();
double distanceL = -((dL / 15) * 23);
double distanceR = -((dR / 15) * 23);

  static float PID_INTEGRAL_LIMIT = 50;
  static int PID_DRIVE_MAX = 127;
  static int PID_DRIVE_MIN = -127;

  float  encoderValueL = 0;
  float  encoderValueR = 0;
  float  pid_Kp = 2.0;
  float  pid_Ki = 0.04;
  float  pid_Kd = 0.0;

  float  pidErrorL = 1000;
  float  pidErrorR = 1000;
  float  pidLastErrorL = 0;
  float  pidLastErrorR = 0;
  float  pidIntegralL = 0;
  float  pidIntegralR = 0;
  float  pidDerivativeL = 0;
  float  pidDerivativeR = 0;
  float  pidDriveL;
  float  pidDriveR;

pidLastErrorL  = 0;
pidLastErrorR  = 0;
pidIntegralL   = 0;
pidIntegralR   = 0;


while(fabs(pidErrorL) > 0.03 || fabs(pidErrorR) > 0.03 ){

  if(fabs(pidErrorL) > 0.03){
    encoderValueL = -DRIVE_LB.rotation(vex::rotationUnits (rev));
    pidErrorL = encoderValueL - distanceL;

  if( pid_Ki != 0 )
                {
                // If we are inside controlable window then integrate the error
                if( fabs(pidErrorL) < PID_INTEGRAL_LIMIT)
                    pidIntegralL = pidIntegralL + pidErrorL;
                     else
                    pidIntegralL = 0;
                }
            else
                pidIntegralL = 0;
            

            // calculate the derivative
            pidDerivativeL = pidErrorL - pidLastErrorL;
            pidLastErrorL  = pidErrorL;
            

            // calculate drive
            pidDriveL = (pid_Kp * pidErrorL) + (pid_Ki * pidIntegralL) + (pid_Kd * pidDerivativeL);
            
            // limit drive
            if( pidDriveL > PID_DRIVE_MAX )
                pidDriveL = PID_DRIVE_MAX;
            if( pidDriveL < PID_DRIVE_MIN )
                pidDriveL = PID_DRIVE_MIN;
                         
  }else{
     pidDriveL = 0;
  }

  if(fabs(pidErrorR) > 0.03){

    encoderValueR = -DRIVE_RB.rotation(vex::rotationUnits (rev));
    pidErrorR = encoderValueR - distanceR;

  if( pid_Ki != 0 )
                {
                // If we are inside controlable window then integrate the error     
                if(fabs(pidErrorR) < PID_INTEGRAL_LIMIT)
                    pidIntegralR = pidIntegralR + pidErrorR;
                else
                    pidIntegralR = 0;
                }
            else
                pidIntegralR = 0;

            // calculate the derivative
            pidDerivativeR = pidErrorR - pidLastErrorR;
            pidLastErrorR  = pidErrorR;


            // calculate drive
            pidDriveR = (pid_Kp * pidErrorR) + (pid_Ki * pidIntegralR) + (pid_Kd * pidDerivativeR);

            // limit drive
            if( pidDriveR > PID_DRIVE_MAX )
                pidDriveR = PID_DRIVE_MAX;
            if( pidDriveR < PID_DRIVE_MIN )
                pidDriveR = PID_DRIVE_MIN;
          
           
              
              

  }else{
pidDriveR = 0;
  }
            // send to motor
            drive(pidDriveL * speed1, pidDriveR * speed2);
            }

}

void pidRamp(){
  RAMP.resetRotation();
    double distance = -1.3;

  static float PID_INTEGRAL_LIMIT = 50;
  static int PID_DRIVE_MAX = 127;
  static int PID_DRIVE_MIN = -127;

  float  encoderValue = 0;
  float  pid_Kp = 2.0;
  float  pid_Ki = 0.04;
  float  pid_Kd = 0.0;

  float  pidError = 1000;
  float  pidLastError = 0;
  float  pidIntegral = 0;
  float  pidDerivative = 0;
  float  pidDrive;

pidLastError  = 0;
pidIntegral   = 0;

while(fabs(pidError) > 0.03 && !controller1.ButtonA.pressing()){

encoderValue = -RAMP.rotation(vex::rotationUnits (rev));


pidError = encoderValue - distance;


if( pid_Ki != 0 )
                {
                // If we are inside controlable window then integrate the error
                if( fabs(pidError) < PID_INTEGRAL_LIMIT )
                    pidIntegral = pidIntegral + pidError;
                else
                    pidIntegral = 0;
                }
            else
                pidIntegral = 0;

            // calculate the derivative
            pidDerivative = pidError - pidLastError;
            pidLastError  = pidError;

            // calculate drive
            pidDrive = (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);

            // limit drive
            if( pidDrive > PID_DRIVE_MAX )
                pidDrive = PID_DRIVE_MAX;
            if( pidDrive < PID_DRIVE_MIN )
                pidDrive = PID_DRIVE_MIN;

            // send to motor
            run(RAMP, pidDrive );
            }
        sleep(750);
}


void pidRamp2(){
  RAMP.resetRotation();
    double distance = -1;

  static float PID_INTEGRAL_LIMIT = 50;
  static int PID_DRIVE_MAX = 127;
  static int PID_DRIVE_MIN = -127;

  float  encoderValue = 0;
  float  pid_Kp = 2.0;
  float  pid_Ki = 0.04;
  float  pid_Kd = 0.0;

  float  pidError = 1000;
  float  pidLastError = 0;
  float  pidIntegral = 0;
  float  pidDerivative = 0;
  float  pidDrive;

pidLastError  = 0;
pidIntegral   = 0;

while(fabs(pidError) > 0.03 && !controller1.ButtonA.pressing()){

encoderValue = -RAMP.rotation(vex::rotationUnits (rev));


pidError = encoderValue - distance;


if( pid_Ki != 0 )
                {
                // If we are inside controlable window then integrate the error
                if( fabs(pidError) < PID_INTEGRAL_LIMIT )
                    pidIntegral = pidIntegral + pidError;
                else
                    pidIntegral = 0;
                }
            else
                pidIntegral = 0;

            // calculate the derivative
            pidDerivative = pidError - pidLastError;
            pidLastError  = pidError;

            // calculate drive
            pidDrive = (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);

            // limit drive
            if( pidDrive > PID_DRIVE_MAX )
                pidDrive = PID_DRIVE_MAX;
            if( pidDrive < PID_DRIVE_MIN )
                pidDrive = PID_DRIVE_MIN;

            // send to motor
            run(RAMP, pidDrive * 0.5 );
            }
        sleep(750);
}

double encL(){
 return 0;
}






int autoColor = 1;
int autoSide = 1;

void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
//Led1.state(100,vex::percentUnits::pct);

std::string color;
std::string side;

  Brain.Screen.drawImageFromFile( "Collingwood_Shield.png" , 0, 0);

lcdButton test(50, 20, 40, 40, "asd", "FF2525","#FFFFFF", 3);
lcdButton test2(120, 20, 40, 40, "asd", "FF2525","#FFFFFF", 3);


lcdButton redFront(50,135,120,40, "Red Run", "#FF2525", "#FFFFFF", 2);//, color(255, 33, 33));
lcdButton redBack(50,190,120,40, "Red Tower", "#FF2525", "#FFFFFF", 2);//, color(255, 33, 33));
lcdButton blueFront(250,135,120,40, "Blue Run", "#2525FF", "#FFFFFF", 2); 
lcdButton blueBack(250,190,120,40, "Blue Tower", "#2525FF", "#FFFFFF", 2);

while(true){



if(blueFront.pressing()){
run(INTAKE_L, 50);
autoColor = 1;
autoSide = 1;
}

if(blueBack.pressing()){
run(INTAKE_L, -50);
autoColor = 1;
autoSide = -1;
}

if(redFront.pressing()){
run(INTAKE_R, 50);
 autoColor = -1;
autoSide = 1;
}

if(redBack.pressing()){
run(INTAKE_R, -50);
autoColor = -1;
autoSide = -1;
}

color = autoColor;
side = autoSide;

test.setText(color);
test2.setText(side);

//const char temp = INTAKE_L.temperature();
//Brain.Screen.printAt(200, 120, temp);

}





}


void autonomous( void ) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
/*
if(autoSide == 1){
  if(autoColor == 1){
    run(INTAKE_L, 100);
    sleep(2000);
    run(INTAKE_L, 0);
  }else{
    run(INTAKE_L, -100);
    sleep(2000);
    run(INTAKE_L, 0);
  }
  
}else{
  if(autoColor == 1){
    run(INTAKE_R, 100);
    sleep(2000);
    run(INTAKE_R, 0);
  }else{
    run(INTAKE_R, -100);
    sleep(2000);
    run(INTAKE_R, 0);
}
}
*/
/*
run(RAMP, 100);
sleep(500);
run(RAMP, 0);
run(INTAKE_L, INTAKE_R, 300);
sleep(1000);
run(INTAKE_L, INTAKE_R, 0);
run(ARM, 200);
sleep(800);
run(ARM, -200);
sleep(600);
run(RAMP, -100);
sleep(300);
run(RAMP, 0);
drive(0, 0);
run(INTAKE_L, INTAKE_R, -300);
*/
pidDrive(2.5, .6, .6);

drive(0,0);

sleep(100);



pidDrive(-3.25, -2.25, 3, 0.75);
//pidDrive(2, 0.5, 1, 0.5);

run(INTAKE_L, INTAKE_R, 0);

drive(0,0);


/*
//THE TURNING PART
pidDrive(-1.3, 1, -1);
drive(0,0);
pidDrive(.5, 1, 1);
drive(0,0);
run(INTAKE_L, INTAKE_R, 100);
run(RAMP, 100);
sleep(500);
//run(INTAKE_L, INTAKE_R, 100);
run(INTAKE_L, INTAKE_R, 0);
sleep(2000);
run(RAMP, 0);
drive(-50, -50);
run(INTAKE_L, INTAKE_R, 0);
sleep(1000);
*/
run(DRIVE_RF, 0);
  run(DRIVE_LF, 0);
  run(DRIVE_LB, 0);
  run(DRIVE_RB, 0);



}




void usercontrol( void ) {
  // User control code here, inside the loop
   int count = 0;


  while (1) {

Led1.state(100,vex::percentUnits::pct);



count++;
//std::string s = "Spiral1.png";
 
//std::string picString;
//picString = pic;


 
 //s.replace(6, 1, picString); 
// s.replace(7, 4, ".png");
//const char* picture = s.c_str();


/*
if(count == 1)
  Brain.Screen.drawImageFromFile( "Spiral1.png" , 0, 0);
if(count == 2)
  Brain.Screen.drawImageFromFile( "Spiral2.png" , 0, 0);
if(count == 3)
  Brain.Screen.drawImageFromFile( "Spiral3.png" , 0, 0);
if(count == 4)
  Brain.Screen.drawImageFromFile( "Spiral4.png" , 0, 0);
if(count == 5)
  Brain.Screen.drawImageFromFile( "Spiral5.png" , 0, 0);
if(count == 6)
  Brain.Screen.drawImageFromFile( "Spiral6.png" , 0, 0);
if(count == 7)
  Brain.Screen.drawImageFromFile( "Spiral7.png" , 0, 0);
if(count == 8)
  Brain.Screen.drawImageFromFile( "Spiral8.png" , 0, 0);
if(count == 9)
  Brain.Screen.drawImageFromFile( "Spiral9.png" , 0, 0);
if(count == 10)
  Brain.Screen.drawImageFromFile( "Spiral10.png" , 0, 0);
if(count == 11)
  Brain.Screen.drawImageFromFile( "Spiral11.png" , 0, 0);
if(count == 12)
  Brain.Screen.drawImageFromFile( "Spiral12.png" , 0, 0);
*/
if(count == 13)
count = 0;







 
  run(DRIVE_RF, (controller1.Axis3.value() - controller1.Axis4.value() - controller1.Axis1.value()) * 1.5 * slowDrive);
  run(DRIVE_LB, (controller1.Axis3.value() - controller1.Axis4.value() + controller1.Axis1.value()) * 1.5 * slowDrive);

  run(DRIVE_LF, (controller1.Axis3.value() + controller1.Axis4.value() + controller1.Axis1.value()) * 1.5 * slowDrive);
  run(DRIVE_RB, (controller1.Axis3.value() + controller1.Axis4.value() - controller1.Axis1.value()) * 1.5 * slowDrive);


if(controller1.ButtonA.pressing())
{
  if(slowDrive == 1){
  slowDrive = 0.7;
  slowInt = 0;
  vex::task::sleep(500);
}else{
  slowDrive = 1;
  slowInt = 0;
  vex::task::sleep(500);
}
}

if(controller1.ButtonR1.pressing()){
  run(ARM, 200);
}else if(controller1.ButtonR2.pressing()){
  run(ARM, -200);
}else{
  ARM.stop(hold);
}


if(controller1.ButtonX.pressing()){
  pidRamp();
  pidRamp2();
}else if(controller1.ButtonB.pressing()){
  run(RAMP, -200);
}else if(controller1.ButtonY.pressing()){
  run(RAMP, 100);
}else{
  RAMP.stop(hold);
}

if(controller1.ButtonL1.pressing()){
    run(INTAKE_L, INTAKE_R, 100);
} else if(controller1.ButtonL2.pressing()){
    run(INTAKE_L, INTAKE_R, -300 );
} else{
    INTAKE_L.stop(hold);
    INTAKE_R.stop(hold);
}




    vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources. 
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    
    //Run the pre-autonomous function. 
    pre_auton();
       
    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }
}
