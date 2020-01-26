package org.firstinspires.ftc.teamcode;

public class Latches {

    public static void move(HardwareBruinBot robot, double position){
        // 1 == down (clamped)
        // 0 == up (open)
        // .5 is half way between 3 o'clock (etc.)
        robot.rightPlatformServo.setPosition(position);
        robot.leftPlatformServo.setPosition(position);
    }

}
