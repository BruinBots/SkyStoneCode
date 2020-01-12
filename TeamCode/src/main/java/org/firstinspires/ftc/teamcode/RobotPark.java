package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "RobotPark", group = "Jack")

public class RobotPark extends LinearOpMode {

    HardwareBruinBot robot = new HardwareBruinBot();


    private ElapsedTime runtime = new ElapsedTime();
    //public boolean found() { return GoldAlignExample.isFound(); }
    //public boolean isAligned() { return detector.getAligned(); }
    //public double getX() { return detector.getXPosition(); }
    public void runOpMode () {




//        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");
//
//        detector = new GoldAlignDetector();
//        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
//        detector.useDefaults();
//
//        // Optional Tuning
//        detector.alignSize = 50; // Was 100 How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
//        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
//        detector.downscale = 0.4; // How much to downscale the input frames
//
//        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
//        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
//        detector.maxAreaScorer.weight = 0.005;
//
//        detector.ratioScorer.weight = 5;
//        detector.ratioScorer.perfectRatio = 1.0;
//
//        detector.enable();
        //Variable setting rotation angle;


        //Initialize hardware;
        robot.init(hardwareMap);


        double fwdSpeed=0.3;  // Forward Speed, Normally 0.1
        double rotate = 0.2; // Rotation Speed
        double strafe = 0.5;  // Strafe Speed







    }


    public void RobotPark(double UsedSensor, double Direction) {


        ElapsedTime holdTimer = new ElapsedTime();
//        CrashDistance = 5
        int CrashDistance = 5;
//        TimeToLine = 4
        int TimeToLine = 4;
        double TimerPause;

//        Start the timer T
        holdTimer.reset();

//        While (rangesensor < CrashDistance && T < TimeToLine) {
        while (UsedSensor < CrashDistance && holdTimer.time() < TimeToLine) {
//            strafe
            gyroStrafe(1,Direction);
//        }
        }
//        stopBot
        stopBot();
//        Pause timer
        TimerPause = holdTimer.seconds();
//        If (rangeSensor < CrashDistance) {
        if (UsedSensor < CrashDistance) {
//            While (rangesensor < CrashDistance) {
            while (UsedSensor < CrashDistance) {
//                Move forward
                gyroStrafe(1,Direction);
//            }
            }
//            stopbot
//            Unpause timer
            holdTimer.reset();
//            holdTimer.seconds() = TimerPause;
//            While (T < TimeToLine) {
            while (holdTimer.time() < (TimeToLine - TimerPause)) {
//                Strafe
                gyroStrafe(1,Direction);
//            }
        }
//            stopbot
            stopBot();
//        }
        }



    }





public void moveBot(double drive, double rotate, double strafe, double scaleFactor)
{
    // This module takes inputs, normalizes them to DRIVE_SPEED, and drives the motors
//        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // How to normalize...Version 3
    //Put the raw wheel speeds into an array
    double wheelSpeeds[] = new double[4];
    wheelSpeeds[0] = drive + strafe - rotate;
    wheelSpeeds[1] = drive - strafe - rotate;
    wheelSpeeds[2] = drive - strafe + rotate;
    wheelSpeeds[3] = drive + strafe + rotate;
    // Find the magnitude of the first element in the array
    double maxMagnitude = Math.abs(wheelSpeeds[0]);
    // If any of the other wheel speeds are bigger, save that value in maxMagnitude
    for (int i = 1; i < wheelSpeeds.length; i++)
    {
        double magnitude = Math.abs(wheelSpeeds[i]);
        if (magnitude > maxMagnitude)
        {
            maxMagnitude = magnitude;
        }
    }
    // Normalize all of the magnitudes to below 1
    if (maxMagnitude > 1.0)
    {
        for (int i = 0; i < wheelSpeeds.length; i++)
        {
            wheelSpeeds[i] /= maxMagnitude;
        }
    }
    // Send the normalized values to the wheels, further scaled by the user
        robot.leftFrontDrive.setPower(scaleFactor * wheelSpeeds[0]);
        robot.leftRearDrive.setPower(scaleFactor * wheelSpeeds[1]);
        robot.rightFrontDrive.setPower(scaleFactor * wheelSpeeds[2]);
        robot.rightRearDrive.setPower(scaleFactor * wheelSpeeds[3]);





}
    public void stopBot()
    {
        // This function stops the robot
        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
    }

    public double sonarDistance (){
        // Returns distance from the sonar sensor over an average of 4 values
        // Trying to get around noise in the sensor
        // 75 is the scaling factor between voltage and distance in INCHES
        // based on data collected on 11/17/2018
        double average;
        average = robot.sonarSensor.getVoltage();
        sleep(1);
        average = average + robot.sonarSensor.getVoltage();
        sleep(1);
        average = average + robot.sonarSensor.getVoltage();
        sleep(1);
        average = average + robot.sonarSensor.getVoltage();
        return (average*75);
    }


    public double rangeSensor (){
        double average;
        average = robot.rangeSensor.getDistance(DistanceUnit.INCH);
        sleep(1);
        average = average + robot.rangeSensor.getDistance(DistanceUnit.INCH);
        sleep(1);
        average = average + robot.rangeSensor.getDistance(DistanceUnit.INCH);
        sleep(1);
        average = average + robot.rangeSensor.getDistance(DistanceUnit.INCH);
        average = average/4;

        //the sensor is reasonably accurate before 6 inches
        //1.1 comes from having the slope being 1.1 from data collected
        if (average > 6) {
            average = average * 1.1;
        }

        return average;
    }


    public void gyroStrafe ( double speed, double heading) {
        // This function will strafe the robot at a given speed while holding a heading

        double error = getError(heading);
        double deadband = 3;
        error = getError(heading);
        if (error < 0 && Math.abs(error) > deadband) {
            // Nagative error greater than 5 degrees, left of desired heading, input positive rotation
            moveBot(0, -.25, speed, 0.3);
        } else if (error > 0 && Math.abs(error) > deadband) {
            // Positive Error greater than 5 degrees, right of desired heading, input negative rotation
            moveBot(0, 0.25, speed, 0.3);
        } else {
            // Robot is on course
            moveBot(0, 0, speed, 0.3);
        }
    }

        public double getError(double targetAngle) {

            double robotError;

            // calculate error in -179 to +180 range  (
            robotError = targetAngle - getHeading();
            while (robotError > 180) { robotError -= 360;}
            while (robotError <= -180) {robotError += 360;}
            return robotError;
        }


        public double getHeading()
        {
            // Get the current heading.

            Orientation angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double heading = -(angles.firstAngle+360)%360;

            if (heading < -180)
                heading += 360;
            else if (heading > 180)
                heading -= 360;

            return heading;

        }


    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.

     */

    public void gyroHoldStopOnTouch( double speed, double angle) {
        // This function drives on a specified heading until the touch sensor is pressed
        double error;
        double PCoeff = 0.1;
        // keep looping while we have time remaining.

        while (robot.backTouchSensor.getState()) {

            telemetry.addData("say", "gyrohold for the touch sensor");
            telemetry.update();
            // Update telemetry & Allow time for other processes to run.
            //error = Range.clip(getError(angle),-0.3,0.3);
            error = PCoeff * getError(angle);
            moveBot(speed, error, 0, 0.2);
        }

        //stop all motion
        stopBot();
    }



    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */

    public void gyroHold( double speed, double angle, double holdTime) {
        // This function drives on a specified heading for a given time
        // Time is in seconds!!!!!
        ElapsedTime holdTimer = new ElapsedTime();
        double error;
        double PCoeff = 0.01;
        // keep looping while we have time remaining.
        holdTimer.reset();
        while ((!isStopRequested() && holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            //error = Range.clip(getError(angle),-0.3,0.3);
            error = PCoeff * getError(angle);
            moveBot(speed, error, 0, 0.3);
        }

        //stop all motion
        stopBot();
    }






    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */


    //holdtime is in seconds
    public void gyroHoldStrafe( double speed, double angle, double strafe, double holdTime) {
        // This function drives on a specified heading for a given time
        // Time is in seconds!!!!!
        ElapsedTime holdTimer = new ElapsedTime();
        double error;
        double PCoeff = 0.1;
        // keep looping while we have time remaining.
        holdTimer.reset();
        while ((!isStopRequested() && holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            //error = Range.clip(getError(angle),-0.3,0.3);
            error = PCoeff * getError(angle);
            moveBot(speed, error, strafe, 0.3);
        }

        //stop all motion
        stopBot();
    }









    }


