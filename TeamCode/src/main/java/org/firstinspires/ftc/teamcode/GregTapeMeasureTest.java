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

    public void runOpMode () {

    //Initialize hardware (REQUIRED)
        robot.init(hardwareMap);

        //reset the encoder on the tape measure (REQUIRED)

        robot.tapeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Wait for the Start button to be pushed
        while (!isStarted()) {
             // Put things to do prior to start in here
        }


        // pattern should be:
        // move out
        // move in
        // move out
        // move out
        // move in
        // move back to 0
        GregTapeMeasure.goToPosition(robot, -800);
        //GregLatches.move(robot, 1); // 1 = down and 0 = up
        sleep(5000);
        stopBot();

        GregTapeMeasure.goToPosition(robot, -200);
        sleep(3000);
        stopBot();

        GregTapeMeasure.goOutOnePortion(robot);

        sleep(3000);
        stopBot();

        GregTapeMeasure.goOutOnePortion(robot);
        sleep(3000);
        stopBot();

        GregTapeMeasure.goInOnePortion(robot);
        sleep(3000);
        stopBot();

        GregTapeMeasure.goToPosition(robot, 0);
        sleep(3000);
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


