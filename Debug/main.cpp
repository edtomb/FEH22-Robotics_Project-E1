#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHMotor.h>

//#include <FEHRPS.h>
#include <FEHServo.h>
#include <FEHBattery.h>
#define LEFTPERCENT 30.25
#define RIGHTPERCENT -37.25
AnalogInputPin cdsSensor(FEHIO::P0_0);
FEHMotor leftMotor(FEHMotor::Motor0,7.2);
FEHMotor rightMotor(FEHMotor::Motor3,7.2);
void driveBackwards(float distance){
    //distance in inches
    float velocity = 6.2;
    //in in/s
    //velocity will be used to calculate
    float driveTime = distance/velocity;
    leftMotor.SetPercent(-LEFTPERCENT);
    rightMotor.SetPercent(-RIGHTPERCENT);
    Sleep(driveTime);
    leftMotor.Stop();
    rightMotor.Stop();
}
void driveForward(float distance){
    //distance in inches
    float velocity = 6.2;
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
    float turnVelocity = 1.5708;
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
    float turnVelocity = 1.5708;
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
    LCD.WriteLine("The robot will drive forward for 3 seconds.");
    leftMotor.SetPercent(LEFTPERCENT);
    rightMotor.SetPercent(RIGHTPERCENT);
    Sleep(3.0);
    leftMotor.Stop();
    rightMotor.Stop();

}
void debugLeft(){
    //2 and 1/2 revs in 10 seconds 
    //1.57 rad/s

    LCD.WriteLine("The robot will turn left for 5 seconds");
    leftMotor.SetPercent(-LEFTPERCENT);
    rightMotor.SetPercent(RIGHTPERCENT);
    Sleep(5.0);
    leftMotor.Stop();
    rightMotor.Stop();
}
void debugRight(){
    LCD.WriteLine("The robot will turn right for 5 seconds");
    leftMotor.SetPercent(LEFTPERCENT);
    rightMotor.SetPercent(-RIGHTPERCENT);
    Sleep(5.0);
    
    leftMotor.Stop();
    rightMotor.Stop();

}
int getLightColor(){
    
        float cdsValue = cdsSensor.Value();
        LCD.WriteLine(cdsValue);
        if(cdsValue>=0.0 && cdsValue<=0.75){
            LCD.WriteLine("RED");
            return 1;
        }else{ 
            LCD.WriteLine("Not RED");
            return 2;
        }
        


}
int main(void)
{
    
    float x,y;
    LCD.WriteLine( Battery.Voltage() );

    
   
    /*
    debugForward();
    Sleep(10.0);
    debugLeft();
    Sleep(5.0);
    debugRight();
    */
   while(!LCD.Touch(&x,&y)){}
   while(LCD.Touch(&x,&y)){}
   debugForward();
   debugLeft();
   debugRight();
   while(1==1){
       getLightColor();
       Sleep(1.5);
   }
    
    rightMotor.Stop();
    leftMotor.Stop();
    
	return 0;
}
