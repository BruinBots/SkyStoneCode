package org.firstinspires.ftc.teamcode;

public class Latches {

    /**
     * @param robot Pass the robot from HardwareBruinBot after you have initialized it
     * @param position The position you want the latches to go:  1 = clamped / down; 0 = up (open)
     */
    public static void move(HardwareBruinBot robot, double position){
        // 1 == down (clamped)
        // 0 == up (open)
        // .5 is half way between 3 o'clock (etc.)
        robot.rightPlatformServo.setPosition(position);
        robot.leftPlatformServo.setPosition(position);
    }

}
