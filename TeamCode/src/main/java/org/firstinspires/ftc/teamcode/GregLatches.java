package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class GregLatches {

    /**
     *
     * <p>
     *     Use to move the latches:
     * </p>
     * <ul>
     *     <li>1 = clamped (down)</li>
     *     <li>0 = up (open)</li>
     *     <li>.5 is half way (~ 3 o'clock)</li>
     * </ul>
     *
     *
     * @param robot Pass the robot from HardwareBruinBot after you have initialized it
     * @param position The position you want the latches to go:  1 = clamped / down; 0 = up (open)
     */
    public static void move(HardwareBruinBot robot, Telemetry _telemetry, double position){
        // 1 == down (clamped)
        // 0 == up (open)
        // .5 is half way between 3 o'clock (etc.)
        Telemetry telemetry = _telemetry;
        robot.rightPlatformServo.setPosition(position);
        robot.leftPlatformServo.setPosition(position);
        telemetry.addData("Latches =", position);
    }

}
