


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
    	

    /*
     burgerServo.SetDegree(110);
     Sleep(2.0);

     trayServo.SetDegree(135.0);
     while(!LCD.Touch(&x,&y)){}

     while (getLightColor() == 2){}
     motion.driveForward(14.5,true);
     //To base of ramp

     Sleep(2.0);
     motion.turn(45.0,RIGHT);


     leftMotor.SetPercent(33.0);
     rightMotor.SetPercent(-40.0);
     motion.driveForward(28.0,true);
     // leftMotor.SetPercent(LEFTPERCENT);
     // rightMotor.SetPercent(RIGHTPERCENT);

     motion.turn(73.0,RIGHT);
     motion.driveForward(9.0,true);
     motion.turn(90.0,LEFT);
     motion.driveForward(19.0,true);
     burgerServo.SetDegree(0.0);
     Sleep(2.0);
     burgerServo.SetDegree(110.0);
     */
    