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

import static java.lang.Math.abs;

@Autonomous(name = "GregLatchesTest", group = "Greg")

// Testing external methods for files [Learning]
// only thing it should do is just move the latches 3 times.
// 1) get it to work directly in code (tested and works)
// 2) get method internal to this class to work (setLatches -- tested and works)
// 3) copy method to a different file to make sure it works (file is Latches; method is move)
public class GregLatchesTest extends LinearOpMode {

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
        robot.rightPlatformServo.setPosition(.1);
        robot.leftPlatformServo.setPosition(.1);

        sleep(5000);

        // put them into a known position using a method existing in this file
        // not a terrible way to do it but if setLatches method is used in any other file
        // and it changes, it has to be changed in every single file and there is a lot of
        // duplication, which is not ideal.
        setLatches(.5);


//        robot.tapeMotor.setTargetPosition(-100);
//        robot.tapeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.tapeMotor.setPower(1);
        sleep(5000);
        stopBot();

        // now try it through moveLatches
        // Best way:  put the method in another file, grouped around different capabilities or
        // sensors or movements or motors or whatever.  Then when you want to change it you make
        // a single change there and it will be changed in all places
        //GregLatches.move(robot, 1); // 1 = down and 0 = up
        GregLatches.move(robot, telemetry, 1);
        telemetry.update();
        sleep(5000);
        stopBot();

        // now reset the Latches to .5 again (strange position to know we have succeeded)
        GregLatches.move(robot, telemetry, 5);
        telemetry.update();
        sleep(5000);
        stop();

        // now reset
        GregLatches.move(robot, telemetry,1);
        telemetry.update();
        //GregLatches.move(robot, .8);

        //GregLatches.move(robot, 1);

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


