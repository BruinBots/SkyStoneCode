package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "TapeMeasurePark", group = "Alex")

    public class TapeMeasurePark extends LinearOpMode {

        HardwareBruinBot robot = new HardwareBruinBot();

        public void runOpMode() {

            //Initialize hardware;
            robot.init(hardwareMap);

            //reset the encoder
            robot.armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.tapeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Wait for the Start button to be pushed
            while (!isStarted()) {
                // Put things to do prior to start in here



            }


                // put them into a known position using a method existing in this file
                // not a terrible way to do it but if setLatches method is used in any other file
                // and it changes, it has to be changed in every single file and there is a lot of
                // duplication, which is not ideal.
                // only going to use GregTapeMeasure
                //   setLatches(.5);


//        robot.tapeMotor.setTargetPosition(-100);
//        robot.tapeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.tapeMotor.setPower(1);
                //   sleep(5000);
//        stopBot();

                // now try it through GregTapeMeasure
                // Best way:  put the method in another file, grouped around different capabilities or
                // sensors or movements or motors or whatever.  Then when you want to change it you make
                // a single change there and it will be changed in all places

                // pattern should be:
                //
               // GregTapeMeasure.goToPosition(robot, -800);




            sleep(23000);

                TapeMeasure.goToPosition(robot, -1250);
//                //GregLatches.move(robot, 1); // 1 = down and 0 = up
//                sleep(5000);
//                stopBot();

//            TapeMeasure.goToPosition(robot, 0);
////            GregLatches.move(robot, 1); // 1 = down and 0 = up
//            sleep(5000);
//            stopBot();
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





            }










































