package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Safe  {



            //        Keepmoving = true
        //    final boolean keepMoving; // remove true and set later

        private static double wheelDiameter = 4;
        private static double wheelCircumference = Math.PI * wheelDiameter;
        private static double ticksPerRotation = 360;
        private static double ticksPerInch = ticksPerRotation / wheelCircumference;

        /**
         * @param maxInchFromWall the maximum you want it to go (unit is in inches)
         * @param minInchFromWall the minimum you want it to go (unit is in inches)
         * @param laserSensorGoal the value for the lasersensor search (unit is in inches)
         */



            public static void forwardsAwayFromWall (HardwareBruinBot robot, Telemetry _telemetry, double maxInchFromWall, double minInchFromWall, double laserSensorGoal, double direction) {

                //negative because forwards is negative and backwards is positive for some reason
                double maxTickFromWall = -(inchToTick(maxInchFromWall));
                double minTicksFromWall = -(inchToTick(minInchFromWall));
                boolean keepMoving = true;
                Telemetry telemetry = _telemetry;

                robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



                gyroHold(robot, -0.6, direction);

                while (keepMoving) {
                    if (robot.leftFrontDrive.getCurrentPosition() < maxTickFromWall) {
                        stopBot(robot);
                        keepMoving = false;
                        telemetry.addData("emergency stop", "");
                    }
                    if ((robot.backDistance.getDistance(DistanceUnit.INCH) > laserSensorGoal) &&
                            (robot.backDistance.getDistance(DistanceUnit.INCH) < (laserSensorGoal * 2)) &&
                            robot.leftFrontDrive.getCurrentPosition() < minTicksFromWall) {
                        stopBot(robot);
                        keepMoving = false;
                        telemetry.addData("normal stop", "");
                    }

                }
                telemetry.addData("maxTickFromWall: ", maxTickFromWall);
                telemetry.addData("minTickFromWall: ", minTicksFromWall);
                telemetry.addData("laserSensor: ", robot.backDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("encoder position: ", robot.leftFrontDrive.getCurrentPosition());
                telemetry.update();

            }






    public static void backwardsTowardWall (HardwareBruinBot robot, Telemetry _telemetry, double maxInchFromWall, double minInchFromWall, double laserSensorGoal) {

        //negative because forwards is negative and backwards is positive for some reason
        double maxTickFromWall = (inchToTick(maxInchFromWall));
        double minTicksFromWall = (inchToTick(minInchFromWall));
        boolean keepMoving = true;

        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Telemetry telemetry = _telemetry;

        gyroHold(robot, 0.6, 0);

        //make sure the numbers work by going through the scenario
        while (keepMoving) {
            if (robot.leftFrontDrive.getCurrentPosition() > maxTickFromWall) {
                stopBot(robot);
                keepMoving = false;
                telemetry.addData("emergency stop", "");
            }
            if ((robot.backDistance.getDistance(DistanceUnit.INCH) < laserSensorGoal) &&
                    robot.leftFrontDrive.getCurrentPosition() > minTicksFromWall) {
                stopBot(robot);
                keepMoving = false;
                telemetry.addData("normal stop", "");
            }

        }

        telemetry.addData("maxTickFromWall: ", maxTickFromWall);
        telemetry.addData("minTickFromWall: ", minTicksFromWall);
        telemetry.addData("laserSensor: ", robot.backDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("encoder position: ", robot.leftFrontDrive.getCurrentPosition());
        telemetry.update();

    }


    private static double inchToTick(double inches){
        double ticks;
        ticks = inches * ticksPerInch;
        return ticks;
    }





    public static void moveBot(HardwareBruinBot robot, double drive, double rotate, double strafe, double scaleFactor)
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

        public static double getError(HardwareBruinBot robot, double targetAngle) {

            double robotError;

            // calculate error in -179 to +180 range  (
            robotError = targetAngle - getHeading(robot);
            while (robotError > 180) { robotError -= 360;}
            while (robotError <= -180) {robotError += 360;}
            return robotError;
        }

    public static double getHeading(HardwareBruinBot robot)
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

        public static void gyroHold(HardwareBruinBot robot, double speed, double angle) {
            // This function drives on a specified heading for a given time
            double error;
            double PCoeff = 0.01;
            // keep looping while we have time remaining.


                // Update telemetry & Allow time for other processes to run.
                //error = Range.clip(getError(angle),-0.3,0.3);
                error = PCoeff * getError(robot, angle);
                moveBot(robot, speed, error, 0, 0.3);

        }

    public static void stopBot(HardwareBruinBot robot)
    {
        // This function stops the robot
        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
    }





            }


