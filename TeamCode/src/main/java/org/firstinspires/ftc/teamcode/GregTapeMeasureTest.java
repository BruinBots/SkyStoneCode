package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "GregTapeMeasureTest", group = "Greg")

// Testing external methods for files [Learning]
// only thing it should do is just move the latches 3 times.
// 1) get it to work directly in code (tested and works)
// 2) get method internal to this class to work (setLatches -- tested and works)
// 3) copy method to a different file to make sure it works (file is Latches; method is move)
public class GregTapeMeasureTest extends LinearOpMode {

    HardwareBruinBot robot = new HardwareBruinBot();

//    private ElapsedTime runtime = new ElapsedTime();
    //public boolean found() { return GoldAlignExample.isFound(); }
    //public boolean isAligned() { return detector.getAligned(); }
    //public double getX() { return detector.getXPosition(); }
    public void runOpMode () {

    //Initialize hardware;
        robot.init(hardwareMap);

        //reset the encoder

        robot.tapeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Wait for the Start button to be pushed
        while (!isStarted()) {
             // Put things to do prior to start in here
        }
        //put them into a known position using direct calls (worst way to do this)
// not safe to move tape this way so I am going to not do it.
//        robot.rightPlatformServo.setPosition(.1);
//        robot.leftPlatformServo.setPosition(.1);

     //   sleep(5000);

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
        // move out
        // move in
        // move out
        // move out
        // move in
        GregTapeMeasure.goToPosition(robot, -300);
        //GregLatches.move(robot, 1); // 1 = down and 0 = up
        sleep(5000);
        stopBot();

        GregTapeMeasure.goToPosition(robot, -200);
        sleep(5000);
        stopBot();

        GregTapeMeasure.goOutOnePortion(robot);
        sleep(5000);
        stopBot();

        GregTapeMeasure.goOutOnePortion(robot);
        sleep(5000);
        stopBot();

        GregTapeMeasure.goInOnePortion(robot);
        sleep(5000);
        stopBot();

        // now reset
        //GregLatches.move(robot, 1);
        //GregLatches.move(robot, .8);

    }



    public void stopBot()
    {
        // This function stops the robot
        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
    }

// set the latches to a specific value
    public void setLatches(double position)
    {
        robot.rightPlatformServo.setPosition(position);
        robot.leftPlatformServo.setPosition(position);

    }

    }


