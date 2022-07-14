#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHMotor.h>


#include <FEHServo.h>
#include <FEHBattery.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <FEHBuzzer.h>
#include <FEHRandom.h>

#include <vector>
#include <string>
// Motor equilibrium percentages, declared globally so they can be accessed inside and outside motion class. 

float LEFTPERCENT = 58.4;
float RIGHTPERCENT = -48.2;

// Line following duration for tray task.  
#define Time_Tray 1.3

#define LINETHRESHHOLD 1.5

// left and right boolean definitions for turning. 
#define LEFT false
#define RIGHT true
/*
Input pins/sensors/motors!
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
FEHServo ticketServo(FEHServo::Servo1);

/*
    Classes and methods
*/

/**
 * @brief Motion class holds functions which have to do with shaft encoding and RPS. 
 *
 * The following functions are included in the motion class:
 * Motion(int revCounts) - constructor for the motion class, takes the number of counts (black/white portions) on the pinwheel
 * void debugEncoderValues(int time) - prints the encoder values T/F for the left and right motors for a given time
 * void driveForwrad(float distance, bool dynamic) - drives the robot forward a given distance, with or without dynamic PID
 * void driveBackwards(float distance) - drives the robot backward a given distance
 * void turn(float degrees, bool dirrection) - turns the robot a given number of degrees, left or right
 * void getRPSInfo(FEHFile *fptr) - gets 10 RPS data points and writes them to the LCD and a file
 * void travelTo(float destX, float destY, bool driveThere) - Uses RPS to align and drive to a given point. driveThere is a boolean (default value=true) which determines whether or not to drive there
 * void align(float heading) - aligns the robot to a given heading
 */
class Motion
{
public:
    // encoder counts per revolution, left and right counts, times driveForward is called.
    int countsPerRev, leftCounts, rightCounts, timesCalled;
    FEHFile *rpsTravelLog;
    // Circumference of wheel = PI*D
    float distPerRev = M_PI * 3.25;
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
        rpsTravelLog = SD.FOpen("rpstrav.txt", "w+");
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
        NVM, old idea im not implementing this. Too slow.
        */
        /**
         timesCalled++;
         char fileNum[7], leftFile[11], rightFile[11];
         sprintf(fileNum, "%d.txt", timesCalled);
         strcpy(leftFile, "lft");
         strcpy(rightFile, "rht");
         strcat(leftFile, fileNum);
         strcat(rightFile, fileNum);
         //FEHFile *leftData = SD.FOpen(leftFile, "w+");
         FEHFile *rightData = SD.FOpen(rightFile, "w+");
         */
        int requiredCounts = (distance / distPerRev) * countsPerRev;
        leftCounts = 0;
        rightCounts = 0;
        float rpsX;
        float rpsY;
        float rpsChange;
        // Times left and right wheel become stuck, used for infinite loop termination.
        int stuckCounts = 0;
        // Current values of left and right digital optosensors
        int leftCurrent = leftEncoder.Value();
        int rightCurrent = rightEncoder.Value();
        // Counts per 0.4 seconds of right and left wheel.
        int rightCPQS = 0, leftCPQS = 0;
        // Interval time used for dynamic robot speed change (DRSC) and data logging interval
        double intervalTime = TimeNow();
        // double dataLogInterval = TimeNow();
        double startTime = TimeNow(), elapsedTime;
        // Start her up
        leftMotor.SetPercent(LEFTPERCENT);
        rightMotor.SetPercent(RIGHTPERCENT);
        rpsX = RPS.X();
        rpsY = RPS.Y();
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
            /*
                // Log left and right counts data to file every 0.1 seconds.
                if (TimeNow() - dataLogInterval >= 0.1)
                {
                    SD.FPrintf(leftData, "%f\t%d\n", TimeNow() - startTime, leftCounts);
                    SD.FPrintf(rightData, "%f\t%d\n", TimeNow() - startTime, rightCounts);
                    dataLogInterval = TimeNow();
                }
            */
            if (TimeNow() - intervalTime >= 0.40)
            {

                /*
                    If right wheel is moving faster than left wheel,
                    increase speed of left wheel by the ratio of rightCounts/leftCounts.
                    If left wheel is moving faster than right wheel,
                    decrease speed of left wheel by ratio leftCounts/rightCounts
                    if speeds are relatively even, will change little, but if speeds are uneven, will change a lot.
                */
                rpsChange = sqrt(pow(RPS.X() - rpsX, 2) + pow(RPS.Y() - rpsY, 2));

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
                    if (rpsChange > 0.4)
                    {
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
        // SD.FClose(leftData);
        // SD.FClose(rightData);
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
        int newRightCounts = 0;
        int newLeftCounts = 0;
        float turnRadius = 4.0;
        float rads = (angle * M_PI) / 180.;
        // dist = r*Theta
        // revs = dist/circumference
        float revsRequired = (turnRadius * rads) / distPerRev;
        int requiredCounts = revsRequired * countsPerRev;
        int leftCurrent = leftEncoder.Value();
        int rightCurrent = rightEncoder.Value();
        int numTimeouts = 0;
        if (direction == LEFT)
        {
            leftMotor.SetPercent((-LEFTPERCENT * 2.0) / 3.0);
            rightMotor.SetPercent((RIGHTPERCENT * 2.0) / 3.);
        }
        else
        {
            leftMotor.SetPercent((LEFTPERCENT * 2.0) / 3.);
            rightMotor.SetPercent((-RIGHTPERCENT * 2.0) / 3.);
        }
        // While average of counts is less than the required number of counts, keep going
        // No differentail changes on turns.
        double intervalTime = 0.4;
        while (((leftCounts + rightCounts) / 2.) <= requiredCounts && numTimeouts <= 4)
        {
            if (leftCurrent != leftEncoder.Value())
            {
                leftCurrent = leftEncoder.Value();
                leftCounts++;
                newLeftCounts++;
            }
            if (rightCurrent != rightEncoder.Value())
            {
                rightCurrent = rightEncoder.Value();
                rightCounts++;
                newRightCounts++;
            }
            if (TimeNow() - intervalTime >= 0.4)
            {
                if (newRightCounts == 0 && newLeftCounts == 0)
                {
                    numTimeouts++;
                }
                intervalTime = TimeNow();
                newRightCounts = 0;
                newLeftCounts = 0;
            }
        }
        leftMotor.Stop();
        rightMotor.Stop();
    }

    /**
     * @brief Gets 10 RPS values on screen touch, writes them to screen and records them on a file.
     *
     */
    void getRPSInfo(FEHFile *fptr)
    {
        float x = 0.0, y = 0.0;
        float xOffset, yOffset, qrAngle, adjustedX, adjustedY;
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
            float heading = RPS.Heading() + 90.0;
            if (heading >= 360.)
            {
                heading -= 360.;
            }
            adjustedX = RPS.X();
            adjustedY = RPS.Y();

            SD.FPrintf(fptr, "Point %d\n", count);
            SD.FPrintf(fptr, "X: %f\n", adjustedX);
            SD.FPrintf(fptr, "Y: %f\n", adjustedY);
            SD.FPrintf(fptr, "Heading: %f\n", heading);
            LCD.WriteLine("Point ");
            LCD.Write(count);
            LCD.WriteLine("X, Y, Heading");
            LCD.WriteLine(adjustedX);
            LCD.WriteLine(" ");
            LCD.WriteLine(adjustedY);
            LCD.WriteLine(" ");
            LCD.WriteLine(heading);
            count++;
            x = 0;
            y = 0;
        }
    }
    /**
     * @brief The holy grail of RPS functions. Step aside Steve Jobs. Aligns to and drives to a point on the course. Then, aligns to a final heading set by the user
     *
     * @param destX The RPS X destination coordinate
     * @param destY The RPS Y destination coordinate
     * @param driveThere Optional. Defaults to true. Set this to false if you want to orient to a point without driving to it.
     */
    void travelTo(float destX, float destY, bool driveThere = true)
    {
        float xi, yi, xf, yf, angleI, angleF, angleTurn, travelDist, xOffset, yOffset, qrAngle;

        bool atDest = false;
        SD.FPrintf(rpsTravelLog, "\n");
        while (RPS.X() == -1. || RPS.Y() == -1.)
        {
        }
        if (RPS.X() == -2 || RPS.Y() == -2)
        {
            LCD.WriteLine("DEADZONE");
            driveBackwards(6.0);
        }
        Sleep(0.4);
        xi = RPS.X();
        yi = RPS.Y();
        angleI = RPS.Heading();

        /*
        Adjust for misalignment of QR code
        */
        angleI = RPS.Heading() + 90.0;
        if (angleI >= 360.)
        {
            angleI -= 360.;
        }

        xi = RPS.X();
        yi = RPS.Y();
        // Think travel dist will work. x,y coords are supposed to be in inches.
        // Update: It does.
        xf = destX - xi;
        yf = destY - yi;
        SD.FPrintf(rpsTravelLog, "\tRan travelTo(%f,%f)\n", destX, destY);
        SD.FPrintf(rpsTravelLog, "I think I am at ( %f, %f ) facing %f deg\n", RPS.X(), RPS.Y(), angleI);
        // Tree to find out which quadrant (xf,yf) is in. atan only returns angles (-pi/2,pi/2)
        if (xf >= 0.)
        {
            // 1st and 4th quadrants
            if (xf == 0.)
            {
                // Xf is zero so can't use arctan. if yf is positive we align to 90 degrees if yf is negative we align to 270 degrees.
                // If yf is also zero, then I guess we're at the destination?
                if (yf > 0.)
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
            else if (yf > 0.)
            {
                // First quadrant. atan functions as desired.
                angleF = atan(yf / xf);
                angleF = angleF * (180.0 / M_PI);
            }
            else if (yf < 0.)
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
            if (yf > 0.)
            {
                // 2nd quadrant(-x+y)
                // 180 - abs(angle) will give desired angle
                angleF = atan(yf / xf);
                angleF = angleF * (180.0 / M_PI);
                angleF = 180.0 - abs(angleF);
            }
            else if (yf < 0.)
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
        // Turn in the optimal direction
        angleTurn = angleI - angleF;

        if (!atDest)
        {

            if (angleTurn > 0.)
            {
                SD.FPrintf(rpsTravelLog, "Must travel to the relative coord ( %f, %f ) and turn %f deg right\n", xf, yf, angleTurn);
                turn(angleTurn, RIGHT);
            }
            else if (angleTurn < 0)
            {
                SD.FPrintf(rpsTravelLog, "Must travel to the relative coord ( %f, %f ) and turn %f deg left\n", xf, yf, abs(angleTurn));
                turn(abs(angleTurn), LEFT);
            }
            Sleep(0.4);
            angleI = RPS.Heading() + 90.0;
            if (angleI >= 360.0)
            {
                angleI -= 360.0;
            }
            // Fine tune orientation to align with dest better.
            int count = 0;
            while ((abs(angleI - angleF) > 6.0 && abs(angleI - angleF) < 353.0) && (count <= 5))
            {
                LCD.WriteLine("ALIGNING");

                if (abs(angleI - angleF) < 180.0)
                {

                    if (angleI > angleF)
                    {

                        turn(4.0, RIGHT);
                    }
                    else
                    {
                        turn(4.0, LEFT);
                    }
                }
                // Handles cases where angleI and angleF are on other sides of the 359-0 degree line.
                else
                {
                    if (angleI > angleF)
                    {
                        turn(4.0, LEFT);
                    }
                    else
                    {
                        turn(4.0, RIGHT);
                    }
                }
                Sleep(0.35);
                angleI = RPS.Heading() + 90.0;
                if (angleI >= 360.0)
                {
                    angleI -= 360.0;
                }
                count++;
            }
            // we are now aligned with our destination.
            // Get new initial position values to make sure that dist traveled is still right after the turn.
            xi = RPS.X();
            yi = RPS.Y();
            xf = destX - xi;
            yf = destY - yi;
            travelDist = sqrt(pow(xf, 2) + pow(yf, 2));
            if (driveThere)
            {
                driveForward(travelDist, true);
            }

            float newAngle = RPS.Heading() + 90.0;
            if (newAngle >= 360.0)
            {
                newAngle -= 360.0;
            }
            Sleep(0.4);
            SD.FPrintf(rpsTravelLog, "I have arrived at (%f,%f) facing %f", RPS.X(), RPS.Y(), newAngle);
        }
        Sleep(0.2);
    }

    /**
     * @brief aligns to a heading on the course
     *
     * @param heading The heading in degrees.
     */
    void align(float heading)
    {
        float turnAngle, currentHeading = RPS.Heading() + 90.0;

        if (currentHeading >= 360)
        {
            currentHeading -= 360;
        }
        turnAngle = currentHeading - heading;
        if (turnAngle > 0)
        {
            turn(turnAngle, RIGHT);
        }
        else
        {
            turn(abs(turnAngle), LEFT);
        }
    }
};

/**
 * @brief LineFollowing class holds functions used for line following and debugging the analog optosensors. 
 * 
 * The following functions are included in the LineFollowing Class:
 * int getSensorState() - returns an integer representing the line detection state
 * void debugOptoValues(double desiredTime) - Prints optosensor voltages for a desired amount of time. 
 * void displayOptoState() - Displays a graphic showing which optosensors are currently detecting a line.
 * void follow(double time) - Follows a line for a desired amount of time.
 * 
 */
class LineFollowing
{
public:
    /**
     *  getSensorState()
        @brief returns integer corresponding to line detection state: Middle: 1 Right: 2 Left: 3 None: 0

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
     * @brief Follows a line for set ammount of time.
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
 * @brief Stores the RPS X and Y coordinates of waypoints
 *
 * The following functions are included in the Waypoint class:
 * void logCoordinates() - Allows the user to manually log essential coordinates right before a run
 */
class Waypoints
{
public:
    float jBoxLEDX, jBoxLEDY;
    float trashX, trashY;
    float redButtonX, redButtonY;
    float blueButtonX, blueButtonY;
    float rampBottomX, rampBottomY;
    float rampTopX, rampTopY;
    float grillX, grillY;
    float topRightWallX;
    float topCenterX, topCenterY;

    // Heading is 135 degrees for levers
    float vanillaLeverX, vanillaLeverY;
    float twistLeverX, twistLeverY;
    float chocolateLeverX, chocolateLeverY;
    float ticketSliderX, ticketSliderY;
    float bottomWallX, bottomWallY;
    float vanillaActualX, vanillaActualY;
    float twistActualX, twistActualY;
    float chocolateActualX, chocolateActualY;
    float stopButtonX, stopButtonY;
    float bottomRightWallX, bottomRightWallY;

    Waypoints(int course)
    {
       
        // Initialize waypoints
        jBoxLEDX = 8.2;
        jBoxLEDY = 22.0;
        trashX = 6.8;
        trashY = 24.0;
        redButtonX = 7.0;
        redButtonY = 15.8;
        blueButtonX = 9.3;
        blueButtonY = 15.8;
        rampBottomX = 15.4;
        rampBottomY = 20.2;
        rampTopX = 17.3;
        rampTopY = 47.9;
        topRightWallX = 28.6;
        topCenterX = 27.4;
        topCenterY = 45.7;
        grillX = 30.8;
        grillY = 61.4;
        vanillaLeverX = 16.3;
        vanillaLeverY = 50.4;
        twistLeverX = 18.2;
        twistLeverY = 52.5;
        chocolateLeverX = 22.;
        chocolateLeverY = 57.0;
        ticketSliderX = 30.8;
        ticketSliderY = 42.7;
        bottomWallX = 28.7;
        bottomWallY = 20.5;
        vanillaActualX = 6.2;
        vanillaActualY = 57.5;
        twistActualX = 9.5;
        twistActualY = 60.5;
        chocolateActualX = 12.8;
        chocolateActualY = 63.5;
        stopButtonX = 28.7;
        stopButtonY = 8.5;
    }

    // Overrides the default coordinates if the team decides to log waypoints before a run.
    void logCoordinates()
    {
        float x, y;
        char fileName[11];
        strcpy(fileName, (char *)RPS.CurrentRegionLetter());
        strcat(fileName, "Self.txt");

        FEHFile *waypointlog = SD.FOpen(fileName, "w+");

       

        LCD.WriteLine("\n\n\n");
        LCD.WriteLine("1) JUKEBOX LED");
        Sleep(1.0);
        LCD.ClearBuffer();
        while (!LCD.Touch(&x, &y))
        {
        }
        while (LCD.Touch(&x, &y))
        {
        }
        jBoxLEDX = RPS.X();
        jBoxLEDY = RPS.Y();
        LCD.WriteLine("Coordinate Logged.");
        LCD.Clear();

        LCD.SetFontColor(RED);
        LCD.WriteLine("\n\n\n");
        LCD.WriteLine("4) RED BUTTON");
        Sleep(1.0);
        LCD.ClearBuffer();
        while (!LCD.Touch(&x, &y))
        {
        }
        while (LCD.Touch(&x, &y))
        {
        }
        redButtonX = RPS.X();
        redButtonY = RPS.Y();
        LCD.WriteLine("Coordinate Logged.");
        LCD.Clear();

        LCD.SetFontColor(BLUE);
        LCD.WriteLine("\n\n\n");
        LCD.WriteLine("5) BLUE BUTTON");
        Sleep(1.0);
        LCD.ClearBuffer();
        while (!LCD.Touch(&x, &y))
        {
        }
        while (LCD.Touch(&x, &y))
        {
        }
        blueButtonX = RPS.X();
        blueButtonY = RPS.Y();
        LCD.WriteLine("Coordinate Logged.");
        LCD.Clear();

        LCD.SetFontColor(WHITE);
   
        LCD.WriteLine("\n\n\n");
        LCD.WriteLine("13) BOTTOM RIGHT WALL");
        Sleep(1.0);
        LCD.ClearBuffer();
        while (!LCD.Touch(&x, &y))
        {
        }
        while (LCD.Touch(&x, &y))
        {
        }
        bottomRightWallX = RPS.X();
        bottomRightWallY = RPS.Y();
        LCD.WriteLine("Coordinate Logged.");
        LCD.Clear();

        LCD.WriteLine("\n\n\n");
        LCD.WriteLine("14) TICKET SLIDER");
        Sleep(1.0);
        LCD.ClearBuffer();
        while (!LCD.Touch(&x, &y))
        {
        }
        while (LCD.Touch(&x, &y))
        {
        }
        ticketSliderX = RPS.X();
        ticketSliderY = RPS.Y();
        LCD.WriteLine("Coordinate Logged.");
        LCD.Clear();
        SD.FPrintf(waypointlog, "0. JukeBox LED from the start button: %f,%f\n", jBoxLEDX, jBoxLEDY);
        SD.FPrintf(waypointlog, "1. Red Button: %f,%f\n", redButtonX, redButtonY);
        SD.FPrintf(waypointlog, "2. Blue Button: %f,%f\n", blueButtonX, blueButtonY);
        SD.FPrintf(waypointlog, "3. Ramp Top: %f,%f\n", rampTopX, rampTopY);
        SD.FPrintf(waypointlog, "4. Twist Lever: %f,%f\n", twistLeverX, twistLeverY);
        SD.FPrintf(waypointlog, "5. Bottom Right Wall: %f,%f\n", bottomRightWallX, bottomRightWallY);
        SD.FPrintf(waypointlog, "6. Ticket Slider: %f,%f\n", ticketSliderX, ticketSliderY);
        SD.FPrintf(waypointlog, "7. Stop Button: %f,%f", stopButtonX, stopButtonY);

        SD.FClose(waypointlog);
        LCD.WriteLine("All Coordinates Logged.");
        Sleep(1.0);

        
    
    }
};

//Sort through a vector of words and output the words sorted from least to greatest to the console.


/**
 * NOTE: THE FOLLOWING OUTDATED FUNCTIONS HAVEBEEN MOVED TO LEGACY.CPP
 * driveBackwards()
 * driveForward()
 * turnRight()
 * turnLeft()
 * debugForward()
 * debugLeft()
 * debugRight()
 */

/**
 * @brief Uses the CDS sensor to get the light color.
 *
 * @return Light Color is Red: 1   Light Color is not red: 0
 */
int getLightColor()
{
    LCD.Clear();
    LCD.SetBackgroundColor(GRAY);
    float cdsValue = cdsSensor.Value();
    LCD.WriteLine(cdsValue);
    if (cdsValue >= 0.0 && cdsValue <= 1.3)
    { // Red
        LCD.SetFontColor(RED);
        LCD.FillCircle(160, 120, 60);
        return 1;
    }
    else
    { // Blue
        LCD.SetFontColor(BLUE);
        LCD.FillCircle(160, 120, 60);
        return 0;
    }
}


int main(void)
{

    float x, y;
    double runStart;
    int jukeBoxColor;
    LCD.WriteLine(Battery.Voltage());
    LineFollowing lineFollow;
    Motion motion(20);

    FEHFile *rpsCoordLog = SD.FOpen("coords.txt", "a+");
    FEHFile *rpsTravelLog = SD.FOpen("rpsTrav.txt", "a+");
    FEHFile *data = SD.FOpen("data.txt", "w+");
    //Servo calibration
    trayServo.SetMin(517);
    trayServo.SetMax(2500);
    burgerServo.SetMin(500);
    burgerServo.SetMax(2225);
    ticketServo.SetMin(500);
    ticketServo.SetMax(2300);

    
    // Set all servos to initial positions
    burgerServo.SetDegree(100.0);
    trayServo.SetDegree(0.0);
    ticketServo.SetDegree(170.0);
    RPS.InitializeTouchMenu();
    int coursenum = RPS.CurrentCourse();
    Waypoints *points = new Waypoints(coursenum);
    points->logCoordinates();
    trayServo.SetDegree(45.0);
    // Get ice cream flavor from rps
    int flavor = RPS.GetIceCream();
    SD.FPrintf(data, "Ice cream flavor: %d\n", flavor);
    LCD.WriteLine("Tap to continue.");
    while (!LCD.Touch(&x, &y))
    {
    }
    while (LCD.Touch(&x, &y))
    {
    }
    
    
    x = 0, y = 0;
    LCD.ClearBuffer();
    LCD.WriteLine("Press to begin");
    while (!LCD.Touch(&x, &y))
    {
    }
    while (LCD.Touch(&x, &y))
    {
    }
    // Wait for run to begin.
    float leftReset = LEFTPERCENT, rightReset = RIGHTPERCENT;

    while (getLightColor() != 1)
    {
    }
    runStart = TimeNow();
    // motion.driveForward(.5, true);
    SD.FPrintf(data, "Run start: %f\n", runStart);
    motion.travelTo(points->jBoxLEDX+10.0, points->jBoxLEDY, true);
    motion.travelTo(points->jBoxLEDX+1.5, points->jBoxLEDY, true);
    
    Sleep(1.0);
    jukeBoxColor=getLightColor();

   
    SD.FPrintf(data, "Jukebox color: %d\n", jukeBoxColor);
    SD.FPrintf(data, "CDS voltage at jukebox: %f\n", cdsSensor.Value());
    // Print found jukebox color, and volage found  to data file
    
    // Align with and travel to trashcan.
    motion.travelTo(points->trashX, points->trashY, false);
    lineFollow.follow(Time_Tray);
    motion.driveBackwards(0.75);
    // Dump the tray
    
    Sleep(0.5);
    motion.turn(8.0,RIGHT);
    trayServo.SetDegree(95.0);
    Sleep(1.0);
    trayServo.SetDegree(0.0);
    motion.driveBackwards(2.0);
    motion.travelTo(RPS.X(), RPS.Y() - 1., false);
    
    motion.driveForward(3.0,true);
    
    


    
    if (jukeBoxColor == 1)
    {
        // motion.travelTo(points->redButtonX,points->redButtonY,false);
        motion.travelTo(points->redButtonX, points->redButtonY);
        LEFTPERCENT=leftReset;
        RIGHTPERCENT=rightReset;
        motion.driveBackwards(1.0);
        motion.travelTo(RPS.X(), RPS.Y() - 1., false);
        motion.driveForward(2.0,false);
       
    }
    else
    {
        // motion.travelTo(points->blueButtonX,points->blueButtonY,false);
        motion.travelTo(points->blueButtonX, points->blueButtonY);
        LEFTPERCENT=leftReset;
        RIGHTPERCENT=rightReset;
        motion.driveBackwards(1.0);
        motion.travelTo(RPS.X(), RPS.Y() - 1., false);
        motion.driveForward(2.0,false);
    }
    LEFTPERCENT=leftReset;
    RIGHTPERCENT=rightReset;
    motion.driveBackwards(4.0);

    /*
    motion.travelTo(29.3,18.1);
    motion.travelTo(28.,27.1);
    motion.travelTo(RPS.X(),RPS.Y()+1.0,false);
    motion.turn(75.0,RIGHT);
    burgerServo.SetDegree(35.0);
    Sleep(1.0);
    motion.turn(90.0,LEFT);
    motion.turn(135.,RIGHT);
    */
    burgerServo.SetDegree(100.0);
    
    // Ramp Base
    motion.travelTo(points->rampBottomX, points->rampBottomY);
    motion.travelTo(RPS.X(), RPS.Y() + 1, false);
    // Top of ramp
    motion.travelTo(points->rampTopX, points->rampTopY);
    LEFTPERCENT = leftReset;
    RIGHTPERCENT = rightReset;
    motion.turn(60.0,RIGHT);
    Sleep(0.3);
    motion.travelTo(31.6, RPS.Y()+0.5);
    motion.turn(70.0,LEFT);
    LEFTPERCENT = leftReset;
    RIGHTPERCENT = rightReset;
    //Dont think this line is nescesary
    //motion.travelTo(RPS.X() + 1, RPS.Y() + 5);
    LEFTPERCENT = leftReset;
    RIGHTPERCENT = rightReset;
    burgerServo.SetDegree(0.0);
    motion.travelTo(points->grillX, points->grillY);
    motion.driveBackwards(0.5);
    motion.turn(15.0, RIGHT);
    //Flip the grill up
    burgerServo.SetDegree(110.0);
    Sleep(1.0);
    

    burgerServo.SetDegree(0.0);
    motion.driveBackwards(1.0);
    ticketServo.SetDegree(130.0);
    motion.turn(90, LEFT);
    Sleep(0.5);
    motion.turn(90, RIGHT);
    motion.driveBackwards(4.0);
    ticketServo.SetDegree(170.0);
    burgerServo.SetDegree(110.0);

    LEFTPERCENT = leftReset;
    RIGHTPERCENT = rightReset;
    //motion.travelTo(points->topCenterX, points->topCenterY);
    
    motion.travelTo(RPS.X()-1,RPS.Y(),false);

    LEFTPERCENT = leftReset;
    RIGHTPERCENT = rightReset;
   

    //motion.travelTo(RPS.X() - 1., RPS.Y() + 1., false);
    burgerServo.SetDegree(110);
    LEFTPERCENT = leftReset;
    RIGHTPERCENT = rightReset;
    flavor=1;
    switch (flavor)
    {
    case 0:
        motion.travelTo(points->vanillaActualX, points->vanillaActualY, false);
        motion.travelTo(points->vanillaLeverX, points->vanillaLeverY);

        break;
    case 1:
        // motion.travelTo(points->twistLeverX,points->twistLeverY,false);
        //motion.travelTo(points->twistActualX, points->twistActualY, false);
        motion.travelTo(points->twistLeverX, points->twistLeverY);
        //motion.travelTo(points->twistActualX, points->twistActualY, false);
        break;
    case 2:

        // motion.travelTo(points->chocolateLeverX,points->chocolateLeverY,false);

        motion.travelTo(points->chocolateLeverX, points->chocolateLeverY);
        motion.travelTo(points->chocolateActualX, points->chocolateActualY, false);
    }
    // At lever
    // align to 135 degrees
    motion.travelTo(RPS.X()-1,RPS.Y()+1,false);

    Sleep(0.5);

    LEFTPERCENT = leftReset;
    RIGHTPERCENT = rightReset;
    motion.driveForward(7.0,true);
    Sleep(0.3);
    trayServo.SetDegree(90.0);
    double iceCreamStart = TimeNow();
    Sleep(1.0);
    trayServo.SetDegree(20.0);
    LEFTPERCENT = leftReset;
    RIGHTPERCENT = rightReset;
    motion.driveBackwards(4.0);
    trayServo.SetDegree(130.0);
    while (TimeNow() - iceCreamStart <= 8.0)
    {
    }
    motion.driveForward(4.0, false);
    trayServo.SetDegree(40.0);
    Sleep(1.0);
    trayServo.SetDegree(130.0);
    motion.driveBackwards(9.0);
    motion.turn(60,RIGHT);
    motion.driveForward(4.0,true);
    trayServo.SetDegree(0.0);
    motion.travelTo(17.6, 43.5);
    motion.travelTo(18.3, 20.6);
    motion.turn(60.0,LEFT);
    LEFTPERCENT = leftReset;
    RIGHTPERCENT = rightReset;
    motion.travelTo(points->bottomRightWallX, RPS.Y()-0.5);
    motion.travelTo(RPS.X(), RPS.Y() + 1.0);
    ticketServo.SetDegree(80.0);
    motion.travelTo(points->ticketSliderX, points->ticketSliderY);
    motion.turn(60.0, LEFT);
    Sleep(2.0);
    motion.turn(60.0, RIGHT);
    motion.driveBackwards(5.0);
    ticketServo.SetDegree(170.0);

    Sleep(0.5);
    motion.travelTo(points->stopButtonX, points->stopButtonY);
    LEFTPERCENT = leftReset;
    RIGHTPERCENT = rightReset;

    // motion.travelTo(RPS.X()+1.0,RPS.Y()-1.0,rpsTravelLog);

    SD.FPrintf(data, "Stopping run at %f\n", TimeNow());
    SD.FPrintf(data, "Total Run time: %f seconds\n", TimeNow() - runStart);

    motion.driveForward(5.0, true);

    SD.FClose(motion.rpsTravelLog);
    SD.FClose(rpsCoordLog);
    SD.FClose(data);

    return 0;
}
