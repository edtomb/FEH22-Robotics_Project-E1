#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHMotor.h>

//#include <FEHRPS.h>
#include <FEHServo.h>
#include <FEHBattery.h>
#include <math.h>
#include <FEHRPS.h>

/*
    Global Definitions

    Naming convention: Capitalized, variables with nonimportant values will be named by step number, option number
    eg: JUKEBOXRED: step 1, option 0, so 10

*/
// Motor equilibrium percentages

float LEFTPERCENT = 39.;
float  RIGHTPERCENT = -47.0;
// parameters for line following functions. Will control how long the function is active for(aka time to button)
#define Time_JUKEBOXRED 3
#define Time_JUKEBOXBLUE 3
#define Time_Tray 3.0

#define LINETHRESHHOLD 1.5

//left and right definitions for shaft encoder turning
#define LEFT false
#define RIGHT true
/*
Input pins/sensors/motors!!!!!!
*/
AnalogInputPin cdsSensor(FEHIO::P1_0);
AnalogInputPin leftOpto(FEHIO::P0_2);
AnalogInputPin midOpto(FEHIO::P0_1);
AnalogInputPin rightOpto(FEHIO::P0_0);

DigitalInputPin leftEncoder(FEHIO::P3_1);
DigitalInputPin rightEncoder(FEHIO::P3_0);


FEHMotor leftMotor(FEHMotor::Motor0, 7.2);
FEHMotor rightMotor(FEHMotor::Motor3, 7.2);
//Servo0 on right end(near tray servo), servo7 on left end
FEHServo trayServo(FEHServo::Servo0);
FEHServo ticketServo(FEHServo::Servo7);


/*
    Classes and methods
*/

/**
 * @brief Motion class holds functions which have to do with shaft encoding and RPS. Initialize with objName(int counts on pinwheel)
 * 
 * As of 3/8/22 RPS isnt a thing on our robot yet, but it would be nice to have. 
 */
class Motion{
    public:
    int countsPerRev, leftCounts, rightCounts;
    //Circumference of wheel = 3.14*D = 3.14*3.25
    float distPerRev = 10.21;
    /**
     * @brief Construct a new Motion object
     * 
     * @param revCounts - number of counts per revolution(aka number of dark spots on pinwheel)
     */
    Motion(int revCounts){
        countsPerRev=revCounts;
        leftCounts=0;
        rightCounts=0;
    }
    /**
     * @brief writes the left and right digital optosensor values to the screen every 2s for a set amount of time
     * 
     * @param time 
     *      -Time in seconds
     */
    void debugEncoderValues(int time){
        double startTime = TimeNow();
        while(TimeNow()-startTime<=time){
            LCD.Clear();
            LCD.WriteLine("LEFT ENCODER VALUE:");
            LCD.WriteLine(leftEncoder.Value());
            LCD.WriteLine("RIGHT ENCODER VALUE:");
            LCD.WriteLine(rightEncoder.Value());
            Sleep(2.0);
        }
    }
    /**
     * @brief implementation of driveForward using shaft encoding. 
     *     will dynamically update left and right motor percentages for 
     *      straight driving.
     * 
     * @param distance
     *          - The desired distance
     * @param dynamic
     *          -TRUE to dynamically change motor speeds.
     *          
     */
    void driveForward(float distance, bool dynamic){
        int requiredCounts = (distance/distPerRev)*countsPerRev;
        leftCounts=0;
        rightCounts=0;
        //Current values of left and right digital optosensors
        int leftCurrent = leftEncoder.Value();
        int rightCurrent = rightEncoder.Value();
        //Counts per half second of right and left wheel.
        int rightCPQS = 0,leftCPQS=0;
        double intervalTime = TimeNow();
        double startTime = TimeNow(),elapsedTime;
        //Start her up
        leftMotor.SetPercent(LEFTPERCENT);
        rightMotor.SetPercent(RIGHTPERCENT);
        //While average of left and right counts are less than counts for a desired distance, continue.
        while(((leftCounts+rightCounts)/2)<requiredCounts){
            //If left optosensor switches from a 0 to a 1 or vice versa, update left current value and add 1 to leftCounts
            if(leftCurrent!=leftEncoder.Value()){
                leftCurrent=leftEncoder.Value();
                leftCounts++;
                leftCPQS++;
            }
            if(rightCurrent!=rightEncoder.Value()){
                rightCurrent=rightEncoder.Value();
                rightCounts++;
                rightCPQS++; 
            }
            //Check difference between right and left counts every half second, adjust motor percentages to compensate for difference
            //To keep speeds bounded, will only increase and decrease the speed of one wheel. 
            if(dynamic){
            if(TimeNow()-intervalTime>=0.25){
                /*
                    If right wheel is moving faster than left wheel,
                    increase speed of left wheel by the ratio of rightCounts/leftCounts.
                    If left wheel is moving faster than right wheel, 
                    decrease speed of left wheel by ratio leftCounts/rightCounts
                    if speeds are relatively even, will change little, but if speeds are uneven, will change a lot.
                */

               //TODO: TESTING WITH THIS TO GET PERCENT CHANGES TO WORK WELL
               //ALSO TODO: IMPLEMENT RPS TO ALIGN WITH DEST.
               if(leftCPQS==0&&rightCPQS>0){
                   //IF motor is running into a wall, thats bad! wow, i know. 
                   //Fix this. 
                        leftMotor.Stop();
                        rightMotor.SetPercent(-RIGHTPERCENT/2.0);
                        Sleep(0.5);
                        leftMotor.SetPercent(LEFTPERCENT);
                        rightMotor.SetPercent(RIGHTPERCENT);
                        //CODE FOR REALIGNING TO DESTINATION WITH HERE.
                    }else if(rightCPQS==0){
                        rightMotor.Stop();
                        leftMotor.SetPercent(-LEFTPERCENT/2.0);
                        Sleep(0.5);
                        leftMotor.SetPercent(LEFTPERCENT);
                        rightMotor.SetPercent(RIGHTPERCENT);
                        //CODE FOR REALIGNING TO DESTINATION WITH HERE.
                        
                    }
                    else{
                if(rightCPQS>leftCPQS){
                    //no nooby divide by 0 errors.
                    //Make Right Smaller, left larger 
                    
                    LEFTPERCENT+=(leftCPQS/(float)rightCPQS);
                    //Right polarity reversed, so add to it to decrease it. 
                    RIGHTPERCENT+=(leftCPQS/(float)rightCPQS);
                    }
                
                else if(leftCPQS>rightCPQS){
                    //Make right larger, left smaller
                    
                    //Ratio of current left percent to right percent times ratio of right counts to left counts
                    //Leftpercent will generally decrease by less in this case, which is fine as that makes sense the way
                    //our motors work. 
                    LEFTPERCENT-=(rightCPQS/(float)leftCPQS);
                    RIGHTPERCENT-=(rightCPQS/(float)leftCPQS);
                }
            }
                //Change left percent to new value, set left and right counts per half sec to 0;
                leftMotor.SetPercent(LEFTPERCENT);
                rightMotor.SetPercent(RIGHTPERCENT);
                rightCPQS=0;
                leftCPQS=0;
                intervalTime = TimeNow();
            }
            }
        }
        leftMotor.Stop();
        rightMotor.Stop();
        elapsedTime = TimeNow()-startTime;
        LCD.Clear();

        LCD.WriteLine("NEW LEFT PERCENT: ");
        LCD.WriteLine(LEFTPERCENT);
        LCD.WriteLine("RIGHT PERCENT:");
        LCD.WriteLine(RIGHTPERCENT);
        LCD.WriteLine("1)Distance driven 2) time(seconds)");
        LCD.WriteLine(distance);
        LCD.WriteLine(elapsedTime);
    }
    /**
     * @brief Shaft encoding implementation of backwards driving. Does not include dynamic speed change capability right now
     * 
     * @param distance 
     *          -The distance desired.
     */
    void driveBackwards(float distance){
        int requiredCounts = (distance/distPerRev)*countsPerRev;
        leftCounts=0;
        rightCounts=0;
        //Current values of left and right digital optosensors
        int leftCurrent = leftEncoder.Value();
        int rightCurrent = rightEncoder.Value();
        //Start time
        double startTime = TimeNow(),elapsedTime;
        //Start her up
        leftMotor.SetPercent(LEFTPERCENT);
        rightMotor.SetPercent(RIGHTPERCENT);
        //While average of left and right counts are less than counts for a desired distance, continue.
        while(((leftCounts+rightCounts)/2)<requiredCounts){
            //If left optosensor switches from a 0 to a 1 or vice versa, update left current value and add 1 to leftCounts
            if(leftCurrent!=leftEncoder.Value()){
                leftCurrent=leftEncoder.Value();
                leftCounts++;
            }
            if(rightCurrent!=rightEncoder.Value()){
                rightCurrent=rightEncoder.Value();
                rightCounts++;
                
            } 
        }
        leftMotor.Stop();
        rightMotor.Stop();
        elapsedTime = TimeNow()-startTime;
        LCD.Clear();

        LCD.WriteLine("1)Distance driven 2) time(seconds)");
        LCD.WriteLine(distance);
        LCD.WriteLine(elapsedTime);
    }
    
    /**
     * @brief shaft encoding implementation of turn left.
     * 
     * @param angle
     *      -The angle for which to turn (in degrees)
     * @param direction
     *      -Use global variable LEFT for left turn and RIGHT for right turn
     */
    void turn(float angle, bool direction){
        /*
        Wheelspan of robot is 
        */
       leftCounts=0;
       rightCounts=0;
       float turnRadius = 3.875;
       float rads = (angle*3.1415)/180;
       //dist = r*Theta
       //revs = dist/circumference
       float revsRequired = (turnRadius*rads)/distPerRev;
       int requiredCounts = revsRequired*countsPerRev;
       int leftCurrent = leftEncoder.Value();
       int rightCurrent = rightEncoder.Value();
       if(direction==LEFT){
           leftMotor.SetPercent(-LEFTPERCENT);
           rightMotor.SetPercent(RIGHTPERCENT);
       }else{
           leftMotor.SetPercent(LEFTPERCENT);
           rightMotor.SetPercent(-RIGHTPERCENT);
       }
       //While average of counts is less than the required number of counts, keep going
       //No differentail changes on turns. 
       while(((leftCounts+rightCounts)/2)<requiredCounts){
           if(leftCurrent!=leftEncoder.Value()){
                leftCurrent=leftEncoder.Value();
                leftCounts++;
            }
            if(rightCurrent!=rightEncoder.Value()){
                rightCurrent=rightEncoder.Value();
                rightCounts++;
            }
       }
       leftMotor.Stop();
       rightMotor.Stop();

    }
    void getRPSInfo(double time){
        double timeStart = TimeNow();

        while(TimeNow()-timeStart<=time){
            LCD.WriteLine("X,Y,Heading");
            LCD.WriteLine(RPS.X());
            LCD.WriteLine(RPS.Y());
            LCD.WriteLine(RPS.Heading());
            Sleep(2.5);


        }
    }
    /**
     * @brief UNUSABLE UNTIL RPS FUNCTIONALITY IS ACHIEVED.
     *        Align to any point on the course using math
     * @param destX - X coordinates of destination
     * @param destY - Y coordinates of destination
     */
    void alignTo(float destX, float destY){
        //Find angle between current heading and desired point using the dot product identity
        //<x,y> = ||x||*||y||*cos(interior angle)
        float currentHeading = RPS.Heading();
        currentHeading = currentHeading*(3.14/180.0);
        //DO arcsin over the cross product if this doesn't work

        float finX = destX-RPS.X();
        float finY = destY-RPS.Y();
        float cHeadingX = cos(currentHeading);
        float cHeadingY = sin(currentHeading);
        float dotprod = (cHeadingX*finX)+(cHeadingY*finY);
        float normOfDestVector = sqrt((finX*finX)+(finY*finY));
        //Look at line on top. Dont need norm of Heading vector because sin^2 + cos^2 is one. 
        float angle = acos(dotprod/normOfDestVector);
        angle = angle*(180.0/3.14);
        //If heading X dest cross product is positive, left turn. If heading X dest cross product is negative, right turn
        float cross = (cHeadingX*finY)-(cHeadingY*finX);
        if(cross>=0){
            turn(angle-90.,LEFT);
        }else{
            turn(angle+90.,RIGHT);
        }

    }  
};
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
    float velocity = 6.0;
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
    float turnVelocity = 1.7;
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
    float turnVelocity = 1.7;
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
    Motion motion(20);
    
    trayServo.SetMin(517);
    trayServo.SetMax(2500);
    ticketServo.SetMin(500);
    ticketServo.SetMax(2225);
    
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
    /*
    trayServo.SetDegree(135.0);
   

    while(!LCD.Touch(&x,&y)){}
    

    //Store and change left and right percent. 
    //Reset left and right percent after tray is dumped so rest of the program runs smoothly.
    
    
    float tempLeft = LEFTPERCENT;
    float tempRight = RIGHTPERCENT;
    LEFTPERCENT = 30.0;
    RIGHTPERCENT = -43.0;
    

    while (getLightColor() == 2){}
    
        motion.turn(30.0,LEFT);
        motion.driveForward(18.5);
        motion.turn(65.0,RIGHT);
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
    driveForward(7.0);
    turnLeft(1.55);
    
    Sleep(1.0);
    
    driveForward(25.0);
    Sleep(1.0);
    //Lower 0.8 Upper 1.20
    turnLeft(1.85);
    Sleep(1.0);
    driveForward(15.0);
    turnLeft(1.57);
    
   
    LCD.WriteLine("SHOULD HAVE GRABBED TICKET");
    Sleep(0.5);
    driveBackwards(8.0);
    */
  // RPS.InitializeTouchMenu();
   
  
   
  
   



   ticketServo.SetDegree(180.);
   Sleep(0.1);
   ticketServo.SetDegree(90);
    trayServo.SetDegree(135.0);
    while(!LCD.Touch(&x,&y)){}
    
    while (getLightColor() == 2){}
    motion.driveForward(14.5,true);
    
    Sleep(2.0);
    motion.turn(30.0,RIGHT);

    
    
    motion.driveForward(24.0,true);
    
    motion.turn(90.0,RIGHT);
    motion.driveForward(4.0,true);
    motion.turn(90.0,LEFT);
    motion.driveForward(15.0,true);
    

    return 0;
}
