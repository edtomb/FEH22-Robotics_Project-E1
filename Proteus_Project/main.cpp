#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHMotor.h>

//#include <FEHRPS.h>
#include <FEHServo.h>
#include <FEHBattery.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <FEHRPS.h>
#include <FEHSD.h>

#include <FEHRandom.h>

/*
    Global Definitions

    Naming convention: Capitalized, variables with nonimportant values will be named by step number, option number
    eg: JUKEBOXRED: step 1, option 0, so 10

*/
// Motor equilibrium percentages

float LEFTPERCENT = 37.1;
float RIGHTPERCENT = -49.0;

// parameters for line following functions. Will control how long the function is active for(aka time to button)
#define Time_JUKEBOXRED 3
#define Time_JUKEBOXBLUE 3
#define Time_Tray 3.0

#define LINETHRESHHOLD 1.5

// left and right definitions for shaft encoder turning
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
// Servo0 on right end(near tray servo), servo7 on left end
FEHServo trayServo(FEHServo::Servo0);
FEHServo burgerServo(FEHServo::Servo7);

/*
    Classes and methods
*/

/**
 * @brief Motion class holds functions which have to do with shaft encoding and RPS. Initialize with objName(int counts on pinwheel)
 *
 * As of 3/8/22 RPS isnt a thing on our robot yet, but it would be nice to have.
 */
class Motion
{
public:
    // encoder counts per revolution, left and right counts, times driveForward is called.
    int countsPerRev, leftCounts, rightCounts, timesCalled;
    // Circumference of wheel = 3.14*D = 3.14*3.25
    float distPerRev = 10.21;
    /**
     * @brief Construct a new Motion object
     *
     * @param revCounts - number of counts per revolution(aka number of dark spots on pinwheel)
     */
    Motion(int revCounts)
    {
        countsPerRev = revCounts;
        leftCounts = 0;
        rightCounts = 0;
        timesCalled = 0;
    }
    /**
     * @brief writes the left and right digital optosensor values to the screen every 2s for a set amount of time
     *
     * @param time
     *      -Time in seconds
     */
    void debugEncoderValues(int time)
    {
        double startTime = TimeNow();
        while (TimeNow() - startTime <= time)
        {
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
     *     straight driving. Logs counts vs time data to a file every time function is called.
     *
     * @param distance
     *          - The desired distance
     * @param dynamic
     *          -TRUE to dynamically change motor speeds.
     *
     */
    void driveForward(float distance, bool dynamic)
    {

        /*
        Create and open rht###.txt and lft###.txt for writing counts data to be graphed in matlab
        Can be called at max 999 times without buffer overflow error.
        */
        timesCalled++;
        char fileNum[7], leftFile[11], rightFile[11];
        sprintf(fileNum, "%d.txt", timesCalled);
        strcpy(leftFile, "lft");
        strcpy(rightFile, "rht");
        strcat(leftFile, fileNum);
        strcat(rightFile, fileNum);
        FEHFile *leftData = SD.FOpen(leftFile, "w+");
        FEHFile *rightData = SD.FOpen(rightFile, "w+");

        int requiredCounts = (distance / distPerRev) * countsPerRev;
        leftCounts = 0;
        rightCounts = 0;
        // Times left and right wheel become stuck, used for infinite loop termination.
        int stuckCounts = 0;
        // Current values of left and right digital optosensors
        int leftCurrent = leftEncoder.Value();
        int rightCurrent = rightEncoder.Value();
        // Counts per 0.4 seconds of right and left wheel.
        int rightCPQS = 0, leftCPQS = 0;
        // Interval time used for dynamic robot speed change (DRSC) and data logging interval
        double intervalTime = TimeNow();
        double dataLogInterval = TimeNow();
        double startTime = TimeNow(), elapsedTime;
        // Start her up
        leftMotor.SetPercent(LEFTPERCENT);
        rightMotor.SetPercent(RIGHTPERCENT);
        // While average of left and right counts are less than counts for a desired distance, continue.
        while (((leftCounts + rightCounts) / 2) < requiredCounts)
        {
            // If left optosensor switches from a 0 to a 1 or vice versa, update left current value and add 1 to leftCounts

            if (leftCurrent != leftEncoder.Value())
            {
                leftCurrent = leftEncoder.Value();
                leftCounts++;
                leftCPQS++;
            }
            if (rightCurrent != rightEncoder.Value())
            {
                rightCurrent = rightEncoder.Value();
                rightCounts++;
                rightCPQS++;
            }

            // Log left and right counts data to file every 0.1 seconds.
            if (TimeNow() - dataLogInterval >= 0.1)
            {
                SD.FPrintf(leftData, "%f\t%d", TimeNow() - startTime, leftCounts);
                SD.FPrintf(rightData, "%f\t%d", TimeNow() - startTime, rightCounts);
                dataLogInterval = TimeNow();
            }

            if (TimeNow() - intervalTime >= 0.40)
            {

                /*
                    If right wheel is moving faster than left wheel,
                    increase speed of left wheel by the ratio of rightCounts/leftCounts.
                    If left wheel is moving faster than right wheel,
                    decrease speed of left wheel by ratio leftCounts/rightCounts
                    if speeds are relatively even, will change little, but if speeds are uneven, will change a lot.
                */

                if (leftCPQS == 0 && rightCPQS == 0)
                {
                    stuckCounts++;
                }
                if (rightCPQS == 0)
                {
                    rightCPQS++;
                }
                if (leftCPQS == 0)
                {
                    leftCPQS++;
                }

                if (dynamic)
                {
                    /**
                     * The math behind the dynamic speed change:
                     * Compensate for count difference by meeting in the middle and adjusting percentages of wheels
                     * Calculation:
                     * Needed CPQS change for each wheel = |rightCPQS-leftCPQS|/2
                     * rightMotor unit CPQS change = rightCPQS/rightpercent
                     * same for left
                     * Needed Percentage change for each wheel = Needed CPQS change/unit CPQS change
                     * As left and right CPQS draw closer together, the percent changes will near zero.
                     * As left and right CPQS become farther apart, the percent changes will increase.
                     */

                    if (rightCPQS > leftCPQS)
                    {

                        // Make Right Smaller, left larger

                        LEFTPERCENT += ((rightCPQS - leftCPQS) * LEFTPERCENT) / (2.0 * leftCPQS);

                        RIGHTPERCENT -= ((rightCPQS - leftCPQS) * RIGHTPERCENT) / (2.0 * rightCPQS);
                    }

                    else if (leftCPQS > rightCPQS)
                    {
                        // Make right larger, left smaller

                        LEFTPERCENT -= ((leftCPQS - rightCPQS) * LEFTPERCENT) / (2.0 * leftCPQS);

                        RIGHTPERCENT += ((leftCPQS - rightCPQS) * RIGHTPERCENT) / (2.0 * rightCPQS);
                    }
                }
                // Change left percent to new value, set left and right counts per half sec to 0;
                leftMotor.SetPercent(LEFTPERCENT);
                rightMotor.SetPercent(RIGHTPERCENT);
                rightCPQS = 0;
                leftCPQS = 0;
                intervalTime = TimeNow();
            }
            // If robot is stuck and remains stuck, terminate.
            if (stuckCounts > 4)
            {
                requiredCounts = 0;
            }
        }

        leftMotor.Stop();
        rightMotor.Stop();
        elapsedTime = TimeNow() - startTime;
        LCD.Clear();
        
        LCD.WriteLine("NEW LEFT PERCENT: ");
        LCD.WriteLine(LEFTPERCENT);
        LCD.WriteLine("RIGHT PERCENT:");
        LCD.WriteLine(RIGHTPERCENT);
        LCD.WriteLine("1)Distance driven 2) time(seconds)");
        LCD.WriteLine(distance);
        LCD.WriteLine(elapsedTime);
        SD.FClose(leftData);
        SD.FClose(rightData);
    }

    /**
     * @brief Shaft encoding implementation of backwards driving. Does not include dynamic speed change capability right now
     *
     * @param distance
     *          -The distance desired.
     */
    void driveBackwards(float distance)
    {
        int requiredCounts = (distance / distPerRev) * countsPerRev;
        leftCounts = 0;
        rightCounts = 0;
        // Current values of left and right digital optosensors
        int leftCurrent = leftEncoder.Value();
        int rightCurrent = rightEncoder.Value();
        // Start time
        double startTime = TimeNow(), elapsedTime;
        // Start her up
        leftMotor.SetPercent(-LEFTPERCENT);
        rightMotor.SetPercent(-RIGHTPERCENT);
        // While average of left and right counts are less than counts for a desired distance, continue.
        while (((leftCounts + rightCounts) / 2) < requiredCounts)
        {
            // If left optosensor switches from a 0 to a 1 or vice versa, update left current value and add 1 to leftCounts
            if (leftCurrent != leftEncoder.Value())
            {
                leftCurrent = leftEncoder.Value();
                leftCounts++;
            }
            if (rightCurrent != rightEncoder.Value())
            {
                rightCurrent = rightEncoder.Value();
                rightCounts++;
            }
        }
        leftMotor.Stop();
        rightMotor.Stop();
        elapsedTime = TimeNow() - startTime;
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
    void turn(float angle, bool direction)
    {
        /*
        Wheelspan of robot is
        */
        leftCounts = 0;
        rightCounts = 0;
        float turnRadius = 3.875;
        float rads = (angle * 3.1415) / 180;
        // dist = r*Theta
        // revs = dist/circumference
        float revsRequired = (turnRadius * rads) / distPerRev;
        int requiredCounts = revsRequired * countsPerRev;
        int leftCurrent = leftEncoder.Value();
        int rightCurrent = rightEncoder.Value();
        if (direction == LEFT)
        {
            leftMotor.SetPercent(-LEFTPERCENT);
            rightMotor.SetPercent(RIGHTPERCENT);
        }
        else
        {
            leftMotor.SetPercent(LEFTPERCENT);
            rightMotor.SetPercent(-RIGHTPERCENT);
        }
        // While average of counts is less than the required number of counts, keep going
        // No differentail changes on turns.
        while (((leftCounts + rightCounts) / 2) < requiredCounts)
        {
            if (leftCurrent != leftEncoder.Value())
            {
                leftCurrent = leftEncoder.Value();
                leftCounts++;
            }
            if (rightCurrent != rightEncoder.Value())
            {
                rightCurrent = rightEncoder.Value();
                rightCounts++;
            }
        }
        leftMotor.Stop();
        rightMotor.Stop();
    }

    /**
     * @brief Gets 5 RPS values on screen touch, writes them to screen and records them on a file.
     *
     */
    void getRPSInfo(FEHFile *fptr)
    {
        float x = 0.0, y = 0.0;
        int count = 1;
        LCD.WriteLine("Logging 10 points to the SD card. After 10 points, getRPSInfo will exit.");

        while (count <= 10)
        {
            while (!LCD.Touch(&x, &y))
            {
            }
            while (LCD.Touch(&x, &y))
            {
            }
            LCD.Clear();
            SD.FPrintf(fptr, "Point %d\n", count);
            SD.FPrintf(fptr, "X: %f\n", RPS.X());
            SD.FPrintf(fptr, "Y: %f\n", RPS.Y());
            SD.FPrintf(fptr, "Heading: %f\n", RPS.Heading());
            LCD.WriteLine("Point ");
            LCD.Write(count);
            LCD.WriteLine("X, Y, Heading");
            LCD.WriteLine(RPS.X());
            LCD.WriteLine(" ");
            LCD.WriteLine(RPS.Y());
            LCD.WriteLine(" ");
            LCD.WriteLine(RPS.Heading());
            count++;
            x = 0;
            y = 0;
        }
    }
    /**
     * @brief The holy grail of functions if you will. Step aside steve jobs. Aligns to and drives to a point on the course.
     *
     * @param destX
     * @param destY
     */
    void travelTo(float destX, float destY, FEHFile *rpsTravelLog)
    {

        float xi, yi, xf, yf, angleI, angleF, angleTurn, travelDist;
        bool atDest = false;
        SD.FPrintf(rpsTravelLog, "\n");
        while (RPS.X() <= -1 || RPS.Y() <= -1)
        {
        }
        xi = RPS.X();
        yi = RPS.Y();
        angleI = RPS.Heading();
        angleI += 90.0;
        if (angleI >= 360)
        {
            angleI -= 360;
        }
        // let (xi,yi) be (0,0). (destX-xi,destY-yi) is distance from that point.
        xf = destX - xi;
        yf = destY - yi;
        // Think travel dist will work. x,y coords are supposed to be in inches.

        SD.FPrintf(rpsTravelLog, "Ran travelTo(%f,%f)\n", destX, destY);
        SD.FPrintf(rpsTravelLog, "I think I am at ( %f, %f ) facing %f deg\n", xi, yi, angleI);
        // Tree to find out which quadrant (xf,yf) is in. atan only returns values (-90,90)
        if (xf >= 0)
        {
            // 1st and 4th quadrants
            if (xf == 0)
            {
                // Ok fun part. Xf is zero so no usey arctan. if yf is positive we align to 90 degrees if yf is negative we align to 270 degrees.
                // If yf is also zero, then I guess we're at the destination?
                if (yf > 0)
                {
                    angleF = 90.0;
                }
                else if (yf < 0)
                {
                    angleF = 270.0;
                }
                else
                {
                    // Well this is akward, we are already where we want to be.(xf,yf)=(xi,yi)=(0,0)
                    angleF = angleI;
                    atDest = true;
                }
            }
            else if (yf > 0)
            {
                // First quadrant. atan functions as desired.
                angleF = atan(yf / xf);
                angleF = angleF * (180.0 / M_PI);
            }
            else if (yf < 0)
            {
                // Fourth quadrant. arctan will return negative values. 360-abs(atan(yf/xf) will give what we want
                angleF = atan(yf / xf);
                angleF = angleF * (180.0 / M_PI);
                angleF = 360.0 - abs(angleF);
            }
            else
            {
                // yf is zero and xf is positive. We are aligning to 0 degrees.
                angleF = 0.0;
            }
        }
        else
        {
            // 2nd and 3rd quadrants
            if (yf > 0)
            {
                // 2nd quadrant(-x+y)
                // 180 - abs(angle) will give desired angle
                angleF = atan(yf / xf);
                angleF = angleF * (180.0 / M_PI);
                angleF = 180.0 - abs(angleF);
            }
            else if (yf < 0)
            {
                // 3rd quadrant(-x,-y)
                // atan will be positive, 180+angle will give desired angle. adding abs() because I can
                angleF = atan(yf / xf);
                angleF = angleF * (180.0 / M_PI);
                angleF = 180.0 + abs(angleF);
            }
            else
            {
                // Y is zero, xf is negative. We are aligning to 180 degrees.
                angleF = 180.0;
            }
        }
        // There is one condition for which we're done here. If we are already where we want to be.

        if (!atDest)
        {
            angleTurn = angleI - angleF;
            if (angleTurn > 0)
            {
                SD.FPrintf(rpsTravelLog, "Must travel to the relative coord ( %f, %f ) and turn %f deg right\n", xf, yf, angleTurn);
                turn(angleTurn, RIGHT);
            }
            else if (angleTurn < 0)
            {
                SD.FPrintf(rpsTravelLog, "Must travel to the relative coord ( %f, %f ) and turn %f deg left\n", xf, yf, abs(angleTurn));
                turn(abs(angleTurn), LEFT);
            }
            // we are now aligned with our destination.
            // drive there.
            xi = RPS.X();
            yi = RPS.Y();
            xf = destX - xi;
            yf = destY - yi;
            travelDist = sqrt((xf * xf) + (yf * yf));
            driveForward(travelDist, true);
            Sleep(1.0);
            float newAngle = RPS.Heading() + 90.0;
            if (newAngle >= 360.0)
            {
                newAngle -= 360.0;
            }
            SD.FPrintf(rpsTravelLog, "I have arrived at (%f,%f) facing %f", RPS.X(), RPS.Y(), newAngle);
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
        }
        else
        {
            return 0;
        }
    }

    /**
     * @brief Prints optosensor values on a loop for a desired amount of time in seconds
     *
     */
    void debugOptoValues(double desiredTime)
    {
        double start = TimeNow();
        while (TimeNow() - start <= desiredTime)
        {
            // lineFollow.displayOptoState();
            LCD.Clear();
            LCD.WriteLine("LEFT OPTO VALUE");
            LCD.WriteLine(leftOpto.Value());
            LCD.WriteLine("MID OPTO VALUE");
            LCD.WriteLine(midOpto.Value());
            LCD.WriteLine("RIGHT OPTO VALUE");
            LCD.WriteLine(rightOpto.Value());
            Sleep(2.5);
            // On Line(LTR): 2.2-2.5 1.3(mid) 2.5+ right
            // Off Line(LTR); .178 .199 .232
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
        int lastState = 1;

        while (TimeNow() - sTime <= followingTime)
        {

            state = getSensorState();
            if (state != 0)
            {
                lastState = state;
            }
            else if (state == 0)
            {
                state = lastState;
            }
            // Display gui of which optosensors robot thinks are over the line.
            if (TimeNow() - guiLoopTime >= 0.5)
            {
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
                leftMotor.SetPercent(LEFTPERCENT + 20.0);
                rightMotor.SetPercent(RIGHTPERCENT + 20.0);
                break;
            case (3):
                // Left sensor is on the line! Correct by driving left
                leftMotor.SetPercent(LEFTPERCENT - 20.0);
                rightMotor.SetPercent(RIGHTPERCENT - 20.0);
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
    // 2.25  revs in 10s
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
    // 2.75 revs in 10s
    float turnVelocity = 1.7;
    float turningTime = angle / turnVelocity;
    // Both percents positi7e: Right forward, left back.

    rightMotor.SetPercent(RIGHTPERCENT);
    leftMotor.SetPercent(-LEFTPERCENT);
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
    FEHFile *rpsCoordLog = SD.FOpen("coords.txt", "a+");
    FEHFile *rpsTravelLog = SD.FOpen("rpsTrav.txt", "a+");

    trayServo.SetMin(517);
    trayServo.SetMax(2500);
    burgerServo.SetMin(500);
    burgerServo.SetMax(2225);

    /*
    All commented out code is being moved to legacy.cpp.
    */

    burgerServo.SetDegree(100.0);
    trayServo.SetDegree(0.0);

    RPS.InitializeTouchMenu();

    while (!LCD.Touch(&x, &y))
    {
    }

    while (LCD.Touch(&x, &y))
    {
    }

    motion.driveForward(15.0, true);
    float leftReset = LEFTPERCENT, rightReset = RIGHTPERCENT;

    while (getLightColor() != 1)
    {
    }
    // Ramp Base
    motion.travelTo(18.0, 20.3, rpsTravelLog);

    // Top of ramp
    motion.travelTo(19.0, 46.9, rpsTravelLog);
    LEFTPERCENT = leftReset;
    RIGHTPERCENT = rightReset;

    Sleep(0.5);
    motion.travelTo(RPS.X(), RPS.Y() + 1, rpsTravelLog);
    motion.travelTo(19.500, 54.1, rpsTravelLog);
    motion.travelTo(RPS.X() - 1., RPS.Y() + 1., rpsTravelLog);
    motion.driveForward(9.0, false);
    trayServo.SetDegree(90.0);
    double iceCreamStart = TimeNow();
    Sleep(0.50);
    motion.driveBackwards(3.0);
    trayServo.SetDegree(130.0);
    while (TimeNow() - iceCreamStart < 8.0)
    {
    }
    motion.driveForward(3.0, false);
    trayServo.SetDegree(0.0);
    Sleep(0.5);
    motion.driveBackwards(9.0);
    motion.travelTo(18.6, 46.9, rpsTravelLog);
    motion.travelTo(18.0, 20.3, rpsTravelLog);
    motion.travelTo(24.56, 2.6, rpsTravelLog);

    /*
    Exploration 03 code: burger flip
    LEFTPERCENT = leftReset;
    RIGHTPERCENT = rightReset;
    motion.travelTo(34.6,RPS.Y(),rpsTravelLog);
    motion.travelTo(RPS.X()+2,RPS.Y()+5,rpsTravelLog);
    burgerServo.SetDegree(0.0);
    motion.travelTo(RPS.X(), 64.5, rpsTravelLog);


    motion.turn(10.0,RIGHT);
    burgerServo.SetDegree(110.0);
    Sleep(2.0);
    rightMotor.Stop();
    burgerServo.SetDegree(0.0);


    motion.turn(90, LEFT);

    burgerServo.SetDegree(110);
    RIGHTPERCENT = RIGHTPERCENT + 10;
    LEFTPERCENT = LEFTPERCENT - 10;
    motion.driveBackwards(10.0);
    motion.travelTo(24.1,49.6,rpsTravelLog);

    motion.travelTo(18.1,55.7,rpsCoordLog);
    motion.travelTo(RPS.X()-1.0,RPS.Y()+1.0,rpsTravelLog);
    motion.driveForward(6.0,false);
    trayServo.SetDegree(100.0);
    Sleep(1.0);
    trayServo.SetDegree(0.0);
    Sleep(0.5);
    motion.turn(10.0,RIGHT);
    trayServo.SetDegree(100.0);
    Sleep(1.0);
    trayServo.SetDegree(0.0);
    Sleep(0.5);
    motion.turn(20.0,LEFT);
    Sleep(1.0);
    trayServo.SetDegree(0.0);
    Sleep(0.5);
    motion.turn(10.0,LEFT);
    trayServo.SetDegree(100.0);
    Sleep(1.0);
    trayServo.SetDegree(0.0);
    Sleep(0.5);
    motion.turn(10.0,LEFT);
    trayServo.SetDegree(100.0);
    */
    SD.FClose(rpsTravelLog);
    SD.FClose(rpsCoordLog);

    return 0;
}
