#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHMotor.h>

//#include <FEHRPS.h>
#include <FEHServo.h>
#include <FEHBattery.h>

/*
    Global Definitions

    Naming convention: Capitalized, variables with nonimportant values will be named by step number, option number
    eg: JUKEBOXRED: step 1, option 0, so 10

*/
// Motor equilibrium percentages
//WILL CHANGE TO 30 and -42 WHEN TRAY IS DUMPED! NVM
float LEFTPERCENT = 32;
float  RIGHTPERCENT = -42.0;
// parameters for line following functions. Will control how long the function is active for(aka time to button)
#define Time_JUKEBOXRED 3
#define Time_JUKEBOXBLUE 3
#define Time_Tray 2.5

#define LINETHRESHHOLD 1.5
/*
Input pins/sensors/motors!!!!!!
*/
AnalogInputPin cdsSensor(FEHIO::P1_0);
AnalogInputPin leftOpto(FEHIO::P0_2);
AnalogInputPin midOpto(FEHIO::P0_1);
AnalogInputPin rightOpto(FEHIO::P0_0);
FEHMotor leftMotor(FEHMotor::Motor0, 7.2);
FEHMotor rightMotor(FEHMotor::Motor3, 7.2);
//Servo0 on right end(near tray servo), servo7 on left end
FEHServo trayServo(FEHServo::Servo0);
FEHServo ticketServo(FEHServo::Servo7);

/*
    Classes and methods
*/
/**
    @brief LineFollowing Class houses the functions used for line following

    Optisensors are mounted very well and dont budge at all. Night and day difference between reflective and non reflective surface. 
    Voltages on reflective are pretty consistent unless front of robot is lifted. The Right Skid could be an issue. 
     Left  : 0.184
     Mid   : 0.131
     Right : 0.195

    Voltages when left sensor is over line:
     Left  : 2.168
     Mid   : 0.132
     Right : 0.201
    Voltages when middle sensor is over line:
     Left  : 0.189
     Mid   : 1.201 or less
     Right : 0.372
    Voltages when right sensor is over line:
     Left  : 0.188
     Mid   : 0.123
     Right : 2.241
    
**/
class LineFollowing
{
public:
    /**
     *  getSensorState()
        @brief returns integer corresponding to line detection state: Middle: 1 Right: 2 Left: 3 None: 0
        @requires
            Optosensors are plugged in to correct ports(see declarations above)
        @ensures
            An integer corresponding to which sensor is on the line to be followed

    **/
    int getSensorState()
    {
       
        
        
        // Check mid optosensor first because mid must be on line.
        if (midOpto.Value() >= 1.1)
        {
            return 1;
        }
        // Nested if tree instead of else if because of GUI capability.
        else if (rightOpto.Value() >= 1.8)
        {
            return 2;
        }
        else if (leftOpto.Value() >= 1.8)
        {
            return 3;
        }else{
            return 0;
        }

    }

    /**
     * @brief Prints optosensor values on a loop for a desired amount of time in seconds
     * 
     */
    void debugOptoValues(double desiredTime){
        double start = TimeNow();
        while (TimeNow()-start<=desiredTime)
    {
        //lineFollow.displayOptoState();
        LCD.Clear();
        LCD.WriteLine("LEFT OPTO VALUE");
        LCD.WriteLine(leftOpto.Value());
        LCD.WriteLine("MID OPTO VALUE");
        LCD.WriteLine(midOpto.Value());
        LCD.WriteLine("RIGHT OPTO VALUE");
        LCD.WriteLine(rightOpto.Value());
        Sleep(2.5);
        //On Line(LTR): 2.2-2.5 1.3(mid) 2.5+ right
        //Off Line(LTR); .178 .199 .232

    }
    }
    /**
     * @brief Displays a graphic showing which optosensors are currently detecting a line. 
     * 
     */
    void displayOptoState()
    {
        bool midOnLine = false, rightOnLine = false, leftOnLine = false;
        if (midOpto.Value() >= 1.1)
        {
            midOnLine = true;
        }
        // Nested if tree instead of else if because of GUI capability.
        if (rightOpto.Value() >= 1.8)
        {
            rightOnLine = true;
        }
        if (leftOpto.Value() >= 1.8)
        {
            leftOnLine = true;
        }
        /*
            Display GUI for sensors currently on line.
        */
       LCD.Clear();
       LCD.SetBackgroundColor(GRAY);
        if (rightOnLine)
        {
            // This means that the left optisensor is on the line. Add this info to the GUI
            LCD.SetFontColor(RED);
            LCD.FillRectangle(75, 25, 50, 100);
        }
        if (midOnLine)
        {
            // Do the same for middle
            LCD.SetFontColor(GREEN);
            LCD.FillRectangle(125, 25, 50, 100);
        }
        if (leftOnLine)
        {
            // Do the same for right
            LCD.SetFontColor(BLUE);
            LCD.FillRectangle(175, 25, 50, 100);
        }
    }
    /**
     * @brief takes in a time integer. Follows that path for set ammount of time. If no path is
     *
     * @param time Path corresponding to global path variable. Determines time for which to follow path.
     */
    void follow(double time)
    {
        
        int state;
        double sTime, followingTime = time, guiLoopTime = TimeNow();
        sTime = TimeNow();

        while (TimeNow() - sTime <= followingTime)
        {

            state = getSensorState();
            //Display gui of which optosensors robot thinks are over the line. 
            if(TimeNow()-guiLoopTime>=0.5){
                displayOptoState();
                guiLoopTime = TimeNow();
            }
            switch (state)
            {
            case (1):
                // Middle sensor is on the line. Want to drive forward in this case
                leftMotor.SetPercent(LEFTPERCENT);
                rightMotor.SetPercent(RIGHTPERCENT);
                break;
            case (2):
                // Right sensor is on the line! Correct by driving Right.
                leftMotor.SetPercent(LEFTPERCENT+20.0);
                rightMotor.SetPercent(RIGHTPERCENT+20.0);
                break;
            case (3):
                // Left sensor is on the line! Correct by driving left
                leftMotor.SetPercent(LEFTPERCENT-20.0);
                rightMotor.SetPercent(RIGHTPERCENT-20.0);
                break;
            case (0):
                // No sensors are on the line. WTF?? Do a circle i guess.
                leftMotor.SetPercent(LEFTPERCENT);
                rightMotor.SetPercent(-RIGHTPERCENT);
            }
        }
        rightMotor.Stop();
        leftMotor.Stop();
    }
};
/**
 * @brief Drives backward a set distance
 *
 * @param distance
 *      The distance to drive(in inches)
 */
void driveBackwards(float distance)
{
    // distance in inches
    float velocity = 6.5;
    // in in/s
    // velocity will be used to calculate
    float driveTime = distance / velocity;
    leftMotor.SetPercent(-LEFTPERCENT);
    rightMotor.SetPercent(-RIGHTPERCENT);
    Sleep(driveTime);
    leftMotor.Stop();
    rightMotor.Stop();
}
/**
 * @brief Drives forward a set distance
 *
 * @param distance
 *      The distance to drive(in inches)
 */
void driveForward(float distance)
{
    // distance in inches
    float velocity = 6.5;
    // in in/s
    // velocity will be used to calculate
    float driveTime = distance / velocity;
    leftMotor.SetPercent(LEFTPERCENT);
    rightMotor.SetPercent(RIGHTPERCENT);
    Sleep(driveTime);
    leftMotor.Stop();
    rightMotor.Stop();
}
/**
 * @brief Turns right over a set angle
 *
 * @param angle
 *      The turn angle(in radian, shadow legends)
 */
void turnRight(float angle)
{
    // Angle in radians.
    // Robot turns from center of 2 wheels, so turning radius is 4.0in
    //2.25  revs in 10s
    float turnVelocity = 2.0;
    // in rad/s

    float turningTime = angle / turnVelocity;
    // Both percents negative: Right back, left forward.
    rightMotor.SetPercent(-RIGHTPERCENT);
    leftMotor.SetPercent(LEFTPERCENT);
    Sleep(turningTime);
    rightMotor.Stop();
    leftMotor.Stop();
}
/**
 * @brief Turns left over a set angle
 *
 * @param angle
 *      The turn angle(in radian, shadow legends)
 */
void turnLeft(float angle)
{
    // Angle in radians.
    // Robot turns from center of 2 wheels, so turning radius is 4.0in
    // S=r*angle, T=S/V
    //2.75 revs in 10s
    float turnVelocity = 1.87;
    float turningTime = angle / turnVelocity;
    // Both percents positi7e: Right forward, left back.

    rightMotor.SetPercent(RIGHTPERCENT );
    leftMotor.SetPercent(-LEFTPERCENT );
    Sleep(turningTime);
    rightMotor.Stop();
    leftMotor.Stop();
}
/**
 * @brief Debugging program. Drives forward for 3 seconds to get measurement for forward velocity
 *
 */
void debugForward()
{

    LCD.Clear();
    LCD.WriteLine("The robot will drive forward for 3 seconds.");
    leftMotor.SetPercent(LEFTPERCENT);
    rightMotor.SetPercent(RIGHTPERCENT);
    Sleep(3.0);
    leftMotor.Stop();
    rightMotor.Stop();
}
/**
 * @brief Debugging program. Turns left for 10 seconds to get left turning velocity.(Revs per sec)
 *
 */
void debugLeft()
{
    // 2 and 1/2 revs in 10 seconds
    // 1.57 rad/s

    LCD.WriteLine("The robot will turn left for 10 seconds");
    leftMotor.SetPercent(-LEFTPERCENT);
    rightMotor.SetPercent(RIGHTPERCENT);
    Sleep(10.0);
    leftMotor.Stop();
    rightMotor.Stop();
}
/**
 * @brief Debugging program. Turns right for 10 seconds to get right turning velocity.(Revs per sec)
 *
 */
void debugRight()
{
    LCD.WriteLine("The robot will turn right for 10 seconds");
    leftMotor.SetPercent(LEFTPERCENT);
    rightMotor.SetPercent(-RIGHTPERCENT);
    Sleep(10.0);

    leftMotor.Stop();
    rightMotor.Stop();
}
/**
 * @brief Uses the CDS sensor to get the light color.
 *
 * @return Light Color is Red: 1   Light Color is not red: 2
 */
int getLightColor()
{
    LCD.Clear();
    float cdsValue = cdsSensor.Value();
    LCD.WriteLine(cdsValue);
    if (cdsValue >= 0.0 && cdsValue <= 0.85)
    {
        LCD.SetBackgroundColor(RED);
        return 1;
    }
    else
    {
        LCD.SetBackgroundColor(BLUE);
        return 2;
    }
}

int main(void)
{

    float x, y;
    LCD.WriteLine(Battery.Voltage());
    LineFollowing lineFollow;
    trayServo.SetMin(517);
    trayServo.SetMax(2500);
    ticketServo.SetMin(500);
    ticketServo.SetMax(2200);
    /*
    /*
    debugForward();
    Sleep(10.0);
    debugLeft();
    Sleep(5.0);
    debugRight();
    *
    // Wait to start
    while (getLightColor() != 1)
    {
    }

    /*
        Go to the jukebox
    **
    // Orient and drive to jukebox. Get light color.
    // TODO: Replace with orient to trash and dump tray
    /*
     turnLeft(.33);
     driveForward(16.5);
     Sleep(0.5);
     int lightColor = getLightColor();
    **

     /*
     Determine which light to press, drive towards correct light.


     if(lightColor==1){
         //Go to red light, return to starting position
         turnLeft(1.25);
         driveForward(4.935);
         driveBackwards(4.935);
         turnLeft(2.16);
     }else{
         //go to blue light
     getLightColor();
     getLightColor();
     turnLeft(1.60);
     driveForward(4.61);
     driveBackwards(4.61);
     turnLeft(1.8);


     //Drive up the ramp

     driveForward(4.25);
     turnLeft(1.);
     Sleep(0.1);
     driveForward(30.0);
     driveBackwards(30.);


     rightMotor.Stop();
     leftMotor.Stop();
     */

    

    /*
        Exploration 3.
    */ 
    
    trayServo.SetDegree(135.0);
    ticketServo.SetDegree(180.0);

    while(!LCD.Touch(&x,&y)){}
    

    //Store and change left and right percent. 
    //Reset left and right percent after tray is dumped so rest of the program runs smoothly.
    
    /*
    float tempLeft = LEFTPERCENT;
    float tempRight = RIGHTPERCENT;
    LEFTPERCENT = 35.0;
    RIGHTPERCENT = -43.0;

    
    
   
    
    while (getLightColor() == 2){}
    
        turnLeft(.5);
        driveForward(20.0);
        turnRight(0.685);
        lineFollow.follow(Time_Tray);
        Sleep(1.0);
        leftMotor.SetPercent(-20);
        Sleep(0.10);
        leftMotor.Stop();
        rightMotor.SetPercent(30);
        Sleep(0.5);

        leftMotor.Stop();
        rightMotor.Stop();
        trayServo.SetDegree(180.0);
        Sleep(0.5);
        trayServo.SetDegree(80.0);
        LCD.Clear();
        LCD.SetFontColor(BLACK);
        LCD.WriteLine("SHOULD HAVE DUMPED TRAY");
    LEFTPERCENT = tempLeft;
    RIGHTPERCENT = tempRight;
    
    driveBackwards(0.5);
    turnRight(3.2);
    driveForward(9.0);
    turnLeft(1.55);
    
    Sleep(1.0);
    
    driveForward(20.0);
    LCD.WriteLine("SHOULD BE BY CENTER OF RAMP");
    Sleep(1.0);
    //Lower 0.8 Upper 1.20
    turnLeft(0.8);
    leftMotor.SetPercent(LEFTPERCENT+5);
    rightMotor.SetPercent(RIGHTPERCENT);
    Sleep(5.0);
    leftMotor.Stop();
    rightMotor.Stop();


    Sleep(0.5);
    turnLeft(0.8);
    LCD.WriteLine("SHOULD BE IN POSITION TO GRAB TICKET");
    Sleep(2.0);
    ticketServo.SetDegree(120);
    LCD.WriteLine("SHOULD HAVE GRABBED TICKET");
    Sleep(0.5);
    driveBackwards(8.0);

    */
   driveForward(15);
   turnLeft(1.85);
   Sleep(0.5);
   driveForward(15);
   turnLeft(1.45);
    
    
    

    

    return 0;
}
