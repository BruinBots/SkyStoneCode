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

@Autonomous(name = "GregTapeTest", group = "Greg")

// Testing external methods for files [Learning]
// only thing it should do is just push out the tape measure a bit.
// Bad name, but now doing just latch
// 1) get method internal to this class to work
// 2) copy method to a different file to make sure it works
public class GregTapeTest extends LinearOpMode {

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
        //put them into a known position
        robot.rightPlatformServo.setPosition(.1);
        robot.leftPlatformServo.setPosition(.1);

        // sleep(7000);


//        robot.tapeMotor.setTargetPosition(-100);
//        robot.tapeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.tapeMotor.setPower(1);
        sleep(1000);
        stopBot();

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


