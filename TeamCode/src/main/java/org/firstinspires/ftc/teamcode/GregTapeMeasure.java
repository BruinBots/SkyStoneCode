package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import org.firstinspires.ftc.robotcore.external.Telemetry;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Func;
//import org.firstinspires.ftc.robotcore.external.Telemetry;




import static java.lang.Math.abs;

public class GregTapeMeasure {

// final means 'can not change -- it is a constant'
    // static means 'usable by methods in this class [static forces memory allocation]
    final static int MAX_TAPEMEASURE_POSITION = 0;
    final static int MIN_TAPEMEASURE_POSITION = -1600;
    final static int OVERDRIVE_VALUE = 8; //
    final static double POWER = 1.0;  // how fast to move the tape measure  Competition should be 1.0; testing = .5
    final static int OUT_PORTION = 200;  // number of encoder ticks to move tapemeasure out
    final static int IN_PORTION = 200;   // number of encoder ticks to move tapemeasure in

    /**
     * Safely goes to a particular position (0 is default/in; -1600 is all the way out).
     * <p>
     *     Typically used during autonomous mode
     * </p>
     * All tape measure position commands should use goToPosition because this does all the
     * error checking (containing soft stops at both ends so we do not extend or retract beyond
     * hardware boundaries)
     *
     * @param robot defined as a HardwareBruinBot
     * @param desiredPosition the position you want to have the tape measure go to
     *                        (0 is all the way in; -1600 is all the way out)
     */
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

    /**
     *
     * Will cause the tape measure to go out one portion (200 encoder ticks, a class constant)
     * <p>
     *     Typically used during tele-op mode (as a command to extend the tape measure)
     * </p>
     *
     * @param robot pass the robot (type HardwareBruinBot) and tape will go out one portion
     *
     */
    public static void goOutOnePortion (HardwareBruinBot robot) {

        int currentPosition = robot.tapeMotor.getCurrentPosition();
        int desiredPosition = currentPosition - OUT_PORTION;
        goToPosition(robot, desiredPosition);

    }

    /**
     *
     * Will cause the tape measure to go in one portion (200 encoder ticks, a class constant)
     * <p>
     *     Typically used during tele-op mode (as a command to retract the tape measure)
     * </p>
     *
     * @param robot pass the robot (type HardwareBruinBot) and tape will go in one portion
     *
     */

    public static void goInOnePortion (HardwareBruinBot robot) {

        int currentPosition = robot.tapeMotor.getCurrentPosition();
        int desiredPosition = currentPosition + IN_PORTION;
        goToPosition(robot, desiredPosition);

    }

}
