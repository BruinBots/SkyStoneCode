package org.firstinspires.ftc.teamcode;

public class Latches {


    /**
     * @param position for positions: 1 is down (clamped) and .5 is up (to the side)
     */

    public static void move(HardwareBruinBot robot, double position) {

        //1 is down, .5 is sideways, and 0 is up
        robot.leftPlatformServo.setPosition(position);
        robot.rightPlatformServo.setPosition(position);
    }


}