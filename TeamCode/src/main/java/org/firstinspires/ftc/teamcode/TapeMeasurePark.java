package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;




    @Autonomous(name = "TapeMeasurePark", group = "Alex")

    public class TapeMeasurePark extends LinearOpMode {

        HardwareBruinBot robot = new HardwareBruinBot();

        public void runOpMode() {

            //Initialize hardware;
            robot.init(hardwareMap);

            //reset the encoder

            robot.tapeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                //GregLatches.move(robot, 1); // 1 = down and 0 = up
                sleep(5000);
                stopBot();

           // TapeMeasure.goToPosition(robot, 0);
            //GregLatches.move(robot, 1); // 1 = down and 0 = up
           // sleep(5000);
            //stopBot();
            }


            public void stopBot ()
            {
                // This function stops the robot
                robot.leftFrontDrive.setPower(0);
                robot.leftRearDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
                robot.rightRearDrive.setPower(0);
            }



            }










































