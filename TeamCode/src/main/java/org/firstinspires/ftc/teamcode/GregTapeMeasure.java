package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.abs;

public class GregTapeMeasure {

// final means 'can not change -- it is a constant'
    // static means 'usable by methods in this class [static forces memory allocation]
    final static int MAX_TAPEMEASURE_POSITION = 0;
    final static int MIN_TAPEMEASURE_POSITION = -1600;
    final static int OVERDRIVE_VALUE = 8; //
    final static double POWER = 0.5;  // how fast to move the tape measure  Competition should be 1.0
    final static int OUT_PORTION = 200;  // number of encoder ticks to move tapemeasure out
    final static int IN_PORTION = 200;   // number of encoder ticks to move tapemeasure in

    // safely move to a position
    public static void goToPosition (HardwareBruinBot robot, int desiredPosition) {

        int currentPosition = robot.tapeMotor.getCurrentPosition();

        if (desiredPosition > MAX_TAPEMEASURE_POSITION) {
            desiredPosition = MAX_TAPEMEASURE_POSITION;
        }
        if (desiredPosition < MIN_TAPEMEASURE_POSITION) {
            desiredPosition = MIN_TAPEMEASURE_POSITION;
        }

        // only move if the difference between desired and current is > overdrive
        // (this is to keep it from vascilating and a bit of safety as well for the motor)
        if (abs(desiredPosition - currentPosition) >= OVERDRIVE_VALUE) {
            robot.tapeMotor.setTargetPosition(desiredPosition);
            robot.tapeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.tapeMotor.setPower(POWER);
        }

        // would be nice to figure out how to use telemetry here

    }

    public static void goOutOnePortion (HardwareBruinBot robot) {

        int currentPosition = robot.tapeMotor.getCurrentPosition();
        int desiredPosition = currentPosition + OUT_PORTION;
        goToPosition(robot, desiredPosition);

    }

    public static void goInOnePortion (HardwareBruinBot robot) {

        int currentPosition = robot.tapeMotor.getCurrentPosition();
        int desiredPosition = currentPosition - IN_PORTION;
        goToPosition(robot, desiredPosition);

    }

}
