#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHMotor.h>

//#include <FEHRPS.h>
#include <FEHServo.h>
#define LEFTPERCENT 31
#define RIGHTPERCENT -37
FEHMotor leftMotor(FEHMotor::Motor0,7.2);
FEHMotor rightMotor(FEHMotor::Motor3,7.2);
AnalogInputPin cdsSensor(FEHIO::P0_0);
void driveForward(float distance){
    //distance in inches
    float velocity = 5.65;
    //in in/s
    //velocity will be used to calculate
    float driveTime = distance/velocity;
    leftMotor.SetPercent(LEFTPERCENT);
    rightMotor.SetPercent(RIGHTPERCENT);
    Sleep(driveTime);
    leftMotor.Stop();
    rightMotor.Stop();
}
void turnRight(float angle){
    //Angle in radians.
    //Robot turns from center of 2 wheels, so turning radius is 4.0in
    //S=r*angle, T=S/V
    float turnVelocity = 1.575;
    //in rad/s
    
    float turningTime = angle/turnVelocity;
    //Both percents negative: Right back, left forward. 
    rightMotor.SetPercent(-RIGHTPERCENT);
    leftMotor.SetPercent(LEFTPERCENT);
    Sleep(turningTime);
    rightMotor.Stop();
    leftMotor.Stop();
}
void turnLeft(float angle){
    //Angle in radians.
    //Robot turns from center of 2 wheels, so turning radius is 4.0in
    //S=r*angle, T=S/V
    float turnVelocity = 1.547;
    float turningTime = angle/turnVelocity;
    //Both percents positive: Right forward, left back. 
    
    rightMotor.SetPercent(RIGHTPERCENT-2.);
    leftMotor.SetPercent(-LEFTPERCENT-2.);
    Sleep(turningTime);
    rightMotor.Stop();
    leftMotor.Stop();
}
void debugForward(){
    //28.25 in 5s with leftpercent 30 rightpercent  -39
    //5.65 in/s
    LCD.Clear();
    LCD.WriteLine("The robot will drive forward for 5 seconds.");
    leftMotor.SetPercent(LEFTPERCENT);
    rightMotor.SetPercent(RIGHTPERCENT);
    Sleep(5.0);
    leftMotor.Stop();
    rightMotor.Stop();

}
void debugLeft(){
    //2 and 1/2 revs in 10 seconds 
    //1.57 rad/s

    LCD.WriteLine("The robot will turn left for 10 seconds");
    leftMotor.SetPercent(-LEFTPERCENT);
    rightMotor.SetPercent(RIGHTPERCENT);
    Sleep(10.0);
    leftMotor.Stop();
    rightMotor.Stop();
}
void debugRight(){
    LCD.WriteLine("The robot will turn right for 10 seconds");
    leftMotor.SetPercent(LEFTPERCENT);
    rightMotor.SetPercent(-RIGHTPERCENT);
    Sleep(10.0);
    
    leftMotor.Stop();
    rightMotor.Stop();

}
int main(void)
{
    
    float x,y,cdsValue;
    

    
    while(!LCD.Touch(&x,&y));
    while(LCD.Touch(&x,&y));
    while(1==1){
        cdsValue = cdsSensor.Value();
        LCD.WriteLine(cdsValue);
        if(cdsValue>=0.599 && cdsValue<=1.4){
            LCD.WriteLine("RED");
        }else if(cdsValue>=2.0 && cdsValue<=2.6){
            LCD.WriteLine("BLUE");
        }
        /*
        No filter:
            Red = 0.599 - 0.750
            Blue = 1.040 - 1.156
        Red filter:
            Red = 0.85-0.90
            Blue= 2.45-2.6
            Ground- No color: 1.9-2.8 if back is covered, Just gonna remove check for this too much inconsistency.
        */

        Sleep(1.5);

    }
}
