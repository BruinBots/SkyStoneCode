/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//imports all important code that we need
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static java.lang.Math.abs;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//the method that defines all of the variables in this program and translates the gamepad inputs to robot outputs
@TeleOp(name="Teleop", group="Pushbot")
public class SkystoneTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareBruinBot robot           = new HardwareBruinBot();   // Use a Pushbot's hardware
    //double          clawOffset      = 0;                       // Servo mid position
    //final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    static final double     COUNTS_PER_MOTOR_REV    = 360 ;

    @Override
    public void runOpMode() {
//        double left;
//        double right;
//        double drive;
//        double turn;
//        double max;





        float drive = 0;
        float strafe = 0;
        float rotate = 0;
        float rampUp = 0;
        float rampDown = 0;
        float clawOpen = 0;
        float clawClose = 0;
//
        boolean lowerArmMotorUp = false;
        boolean lowerArmMotorDown = false;
        boolean intakeFront = false;
        boolean intakeBack = false;
        boolean upperArmMotorOut = false;
        boolean upperArmMotorIn = false;

        boolean drivespeed = true;
        boolean clawRest = true;
        boolean capstoneOut = false;
        boolean capstoneIn = false;
        boolean armOut = false;
        boolean armIn = false;
        boolean armUp = false;
        boolean armDown = false;
        boolean platformServoDown = false;
        boolean platformServoUp = false;
      //  boolean tapeOut = false;
        //boolean tapeIn = false;

        int currentArmLiftPosition =0;  // Used to store the currently commanded arm position
        int MAX_LIFTARM_POSITION = 460;  // ABout 70 steps from arm starting position to full extension
        int currentArmExtendPosition = 0;
        int MAX_ARMEXTEND_POSITION = 0;
        int MIN_ARMEXTEND_POSITION = -250;  //min encoder value is actually -288
//        int MAX_TAPEMEASURE_POSITION = 0;
//        int MIN_TAPEMEASURE_POSITION = -1600;
//        int currentTapeMeasurePosition = 0;





        ElapsedTime runtime = new ElapsedTime();

        waitForStart();



        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

//        reset the encoders
        // reset the armLiftMotor encoder
        robot.armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //reset the armExtendMotor encoder
        robot.armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //reset the tapeMotor
        robot.tapeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Get PID constants
        int motorIndex = ((robot.armLiftMotor).getPortNumber());
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)robot.armLiftMotor.getController();
        PIDCoefficients pidModified = motorControllerEx.getPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);


        // change coefficients using methods included with DcMotorEx class.
        PIDCoefficients pidNew = new PIDCoefficients(4, 2, 3);
        motorControllerEx.setPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);






        int motorIndexLeftFrontWheel = ((robot.leftFrontDrive).getPortNumber());
        DcMotorControllerEx motorControllerExLeftFrontWheel = (DcMotorControllerEx)robot.leftFrontDrive.getController();
        PIDCoefficients pidModifiedLeftFrontWheel = motorControllerExLeftFrontWheel.getPIDCoefficients(motorIndexLeftFrontWheel, DcMotor.RunMode.RUN_USING_ENCODER);


        // change coefficients using methods included with DcMotorEx class.
//        PIDCoefficients pidNewWheels = new PIDCoefficients(4, 2, 3);
//        motorControllerExWheels.setPIDCoefficients(motorIndexWheels, DcMotor.RunMode.RUN_USING_ENCODER, pidNewWheels);




        int motorIndexWheels = ((robot.leftRearDrive).getPortNumber());
        DcMotorControllerEx motorControllerExWheels = (DcMotorControllerEx)robot.leftRearDrive.getController();
        PIDCoefficients pidModifiedWheels = motorControllerExWheels.getPIDCoefficients(motorIndexWheels, DcMotor.RunMode.RUN_USING_ENCODER);


        // change coefficients using methods included with DcMotorEx class.
//        PIDCoefficients pidNewWheels = new PIDCoefficients(4, 2, 3);
//        motorControllerExWheels.setPIDCoefficients(motorIndexWheels, DcMotor.RunMode.RUN_USING_ENCODER, pidNewWheels);




        telemetry.addData("P,I,D (modified) (wheels)", "%.04f, %.04f, %.04f",
                pidModifiedWheels.p, pidModifiedWheels.i, pidModifiedWheels.d);







//        // Get PID constants
//        int motorIndexTape = ((robot.tapeMotor).getPortNumber());
//        DcMotorControllerEx motorControllerExTape = (DcMotorControllerEx)robot.tapeMotor.getController();
//        PIDCoefficients pidModifiedTape = motorControllerEx.getPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//        // change coefficients using methods included with DcMotorEx class.
//        PIDCoefficients pidNewTape = new PIDCoefficients(10, 1, 2);
//        motorControllerExTape.setPIDCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidNewTape);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        TapeMeasure.goToPosition(robot, 0);


            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                // Telemetry Section ------------------------------------------------------------------
                //telemetry.addData("Lift Encoder:", robot.landerLatchLift.getCurrentPosition());
                //telemetry.addData("Arm Encoder:", robot.armRotate.getCurrentPosition());
                //telemetry.update();
                if (isStopRequested()) {
                    stop();
                    sleep(5000);
                }



                // DRIVING SECTION!!!! ----------------------------------------------------------------
                drive = gamepad2.left_stick_y;// Negatieve because the gamepad is weird
                strafe = -gamepad2.left_stick_x;
                rotate = gamepad2.right_stick_x;





                if (gamepad2.a) {
                    moveBot(drive, rotate, strafe, 0.4);

                }
                else if (!gamepad2.a) {
                    moveBot(drive,rotate,strafe,0.7);
                }






                //arm extension section
                //Dpad left moves it out and right moves it in

                //arm lifting section
                //Dpad up moves it up and down moves it down
                //negative is forward
                armOut = gamepad2.dpad_right;
                armIn = gamepad2.dpad_left;
                // Only change value if arm is near commanded value, prevents overdriving arm.  8 seems to work...
                if (abs(currentArmExtendPosition-robot.armExtendMotor.getCurrentPosition()) < 6){
                    if (armIn) {
                        currentArmExtendPosition += 50; // Add 10 to the current arm position
                        if (currentArmExtendPosition > MAX_ARMEXTEND_POSITION) {
                            currentArmExtendPosition = MAX_ARMEXTEND_POSITION; // DOn't let it go highter than Max Position
                        }
                    } else {
                        if (armOut) {
                            currentArmExtendPosition -= 50; // Subtract 10 from the current arm position
                            if (currentArmExtendPosition < MIN_ARMEXTEND_POSITION) {
                                currentArmExtendPosition = MIN_ARMEXTEND_POSITION;  // Don't let it go lower than 0
                            }
                        }
                    }
                }

                telemetry.addData("Current Commanded Pos (for ArmExtend): ",currentArmExtendPosition);
                telemetry.addData("Actual Pos (for ArmExtend): ",robot.armExtendMotor.getCurrentPosition());
                robot.armExtendMotor.setTargetPosition(currentArmExtendPosition);
                robot.armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armExtendMotor.setPower(.9);  //small spool: power 1, big spool: power .5

//                telemetry.update();




//                armOut = gamepad2.dpad_left;
//                armIn = gamepad2.dpad_right;
//                if (armIn) {
//                    robot.armExtendMotor.setPower(.5);
//                    telemetry.addData("armExtend Position", robot.armExtendMotor.getCurrentPosition());
//                }
//                else {
//                    robot.armExtendMotor.setPower(0);
//                }
//
//                if (armOut) {
//                    robot.armExtendMotor.setPower(-.5);
//                    telemetry.addData("armExtend Position", robot.armExtendMotor.getCurrentPosition());
//                }
//                else {
//                    robot.armExtendMotor.setPower(0);
//                }
//                telemetry.update();



                //arm lifting section
                //Dpad up moves it up and down moves it down
                armDown = gamepad2.dpad_down;
                armUp = gamepad2.dpad_up;
                // Only change value if arm is near commanded value, prevents overdriving arm.  8 seems to work...
                if (abs(currentArmLiftPosition-robot.armLiftMotor.getCurrentPosition()) < 8){
                    if (armUp) {
                        currentArmLiftPosition += 150; // Add 10 to the current arm position
                        if (currentArmLiftPosition > MAX_LIFTARM_POSITION) {
                            currentArmLiftPosition = MAX_LIFTARM_POSITION; // DOn't let it go highter than Max Position
                        }
                    } else {
                        if (armDown) {
                            currentArmLiftPosition -= 100; // Subtract 10 from the current arm position
                            if (currentArmLiftPosition < -20) {
                                currentArmLiftPosition = -20;  // Don't let it go lower than 0
                            }
                        }
                    }
                }

                telemetry.addData("Current Commanded Pos: ",currentArmLiftPosition);
                telemetry.addData("Actual Pos: ",robot.armLiftMotor.getCurrentPosition());
                robot.armLiftMotor.setTargetPosition(currentArmLiftPosition);
                robot.armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armLiftMotor.setPower(1);

                //PID value telemetry
                telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                        pidModified.p, pidModified.i, pidModified.d);

//                telemetry.update();
                /*
                if (armUp) {
                    robot.armLiftMotor.setPower(1);
                }
                else {
                    robot.armLiftMotor.setPower(0);
                }

                if (armDown) {
                    robot.armLiftMotor.setPower(-1);
                }
                else {
                    robot.armLiftMotor.setPower(0);
                }
*/

                //Tapemeasure section


                if (gamepad2.right_bumper) {
                    TapeMeasure.goInOnePortion(robot);
                }
                 if (gamepad2.left_bumper) {
                     TapeMeasure.goOutOnePortion(robot);
                 }


//                tapeIn = gamepad2.right_bumper;
//                tapeOut = gamepad2.left_bumper;
//                // Only change value if arm is near commanded value, prevents overdriving arm.  8 seems to work...
//                if (abs(currentTapeMeasurePosition-robot.tapeMotor.getCurrentPosition()) < 8){
//                    if (tapeOut) {
//                        currentTapeMeasurePosition += 400; // Add 10 to the current arm position
//                        if (currentTapeMeasurePosition > MAX_TAPEMEASURE_POSITION) {
//                            currentTapeMeasurePosition = MAX_TAPEMEASURE_POSITION; // DOn't let it go highter than Max Position
//                        }
//                    } else {
//                        if (tapeIn) {
//                            currentTapeMeasurePosition -= 400; // Subtract 10 from the current arm position
//                            if (currentTapeMeasurePosition < MIN_TAPEMEASURE_POSITION) {
//                                currentTapeMeasurePosition = MIN_TAPEMEASURE_POSITION;  // Don't let it go lower than 0
//                            }
//                        }
//                    }
//                }
//
//
//                robot.tapeMotor.setTargetPosition(currentTapeMeasurePosition);
//                robot.tapeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.tapeMotor.setPower(1);
//
//                telemetry.addData("Current Commanded Pos (tape): ",currentTapeMeasurePosition);
//                telemetry.addData("Actual Pos (tape): ",robot.tapeMotor.getCurrentPosition());

//                telemetry.update();






                //Claw Section
                // Read the triggers and roll the Servos
                //right trigger is open and left is closed.
                //open is 1 and close is  .1
                clawOpen = gamepad2.right_trigger;
                clawClose = gamepad2.left_trigger;
                if (clawOpen > clawClose) {
                    robot.clawServo.setPosition(1);
                }
                if (clawOpen < clawClose) {
                    robot.clawServo.setPosition(.1);
                }
                //no = case yet
                //testing

                //platform servo section
                //x is down and b is up
                platformServoDown = gamepad2.x;
                platformServoUp = gamepad2.b;
                if (platformServoDown) {
                    Latches.move(robot, 1);
                }
                else {
                    if (platformServoUp) {
                        Latches.move(robot, 0);
                    }
                }

                // Capstone servo section
                // Pressing Y will dump the capstone then reset
                if (gamepad2.y){
                    robot.capstoneServo.setPosition(0);
                    sleep(600);
                    robot.capstoneServo.setPosition(.8);
                }
                else {
                    robot.capstoneServo.setPosition(.8);
                }




telemetry.update();






                //FIXME: commented out ramp
//                //RAMP SECTION
//
//                // Read the triggers and roll the Servos
//                //one servo as of 10/20/19
//                rampUp = gamepad2.right_trigger;
//                rampDown = gamepad2.left_trigger;
//                if (rampUp > rampDown) {
////                    robot.rampServoRight.setPower(-rampUp);
//                    robot.rampServoLeft.setPower(rampUp);
//                    telemetry.addData("say", "right trigger: rampUp");
//
//                }

                //FIXME: commented out ramp
//                if (rampUp < rampDown) {
//
////                    robot.rampServoRight.setPower(rampDown);
//                    robot.rampServoLeft.setPower(-rampDown);
//                    telemetry.addData("say", "left trigger: rampDown");
//                }
//                if ( rampUp == rampDown) {
//                    telemetry.addData("say", "rampUp = rampDown");
//                }
//                telemetry.update();

//                telemetry.addData("upper arm Encoder:", robot.upperArmMotor.getCurrentPosition());
//                telemetry.addData("lower arm Encoder:", robot.lowerArmMotor.getCurrentPosition());
//                telemetry.update();
//                telemetry.addData("lower arm Encoder:", robot.upperArmMotor.getCurrentPosition());


                //FIXME: commented out lower arm
//                //LOWER ARM PLACE
//                lowerArmMotorUp = gamepad2.dpad_up;
//                lowerArmMotorDown = gamepad2.dpad_down;
////                telemetry.addData("say", "at the lower arm motor place");
////                telemetry.update();
//                if (lowerArmMotorUp) {
//                    robot.lowerArmMotor.setPower(1);
//                } else {
//                    robot.lowerArmMotor.setPower(0);
//
//                }
//
//                if (lowerArmMotorDown) {
//                    robot.lowerArmMotor.setPower(-1);
//                } else {
//                    robot.lowerArmMotor.setPower(0);
//                }

                //FIXME: commented out upper arm
//                //UPPER ARM PLACE
//                upperArmMotorOut = gamepad2.dpad_right;
//                upperArmMotorIn = gamepad2.dpad_left;
////                telemetry.addData("say", "at the upper arm motor place");
////                telemetry.update();
//
//                if (upperArmMotorOut) {
//                    robot.upperArmMotor.setPower(1);
//                } else {
//                    robot.upperArmMotor.setPower(0);
//
//                }
//
//                if (upperArmMotorIn) {
//
//                    robot.upperArmMotor.setPower(-1);
//                } else {
//                    robot.upperArmMotor.setPower(0);
//                }


                //FIXME: commented out intake
//                //INTAKE PLACE
//                intakeFront = gamepad2.left_bumper;
//                intakeBack = gamepad2.right_bumper;
//                if (intakeBack) {
//                    robot.intakeLeft.setPower(1);
//                    robot.intakeRight.setPower(-1);
//                    telemetry.addData("say", "right bumper: intakeBack");
//                }
//                else {
//                    robot.intakeLeft.setPower(0);
//                    robot.intakeRight.setPower(0);
//                }
//                if (intakeFront) {
//                    robot.intakeLeft.setPower(-1);
//                    robot.intakeRight.setPower(1);
//                    telemetry.addData("say", "left bumper: intakeFront");
//                }
//                else {
//                    robot.intakeLeft.setPower(0);
//                    robot.intakeRight.setPower(0);
//                }
//                telemetry.update();

//        if (gamepad2.a) {
//            clawOpen=true;
//            clawClose=false;
//            clawRest=false;
//        }
//
//        if (gamepad2.y) {
//            clawClose=true;
//            clawOpen=false;
//            clawRest=false;
//        }
//
//        if (!gamepad2.a && !gamepad2.y) {
//            clawRest=true;
//            clawOpen=false;
//            clawClose=false;
//        }
//
//        if (clawOpen) {
//            robot.clawMotor.setPower(1);
//        }
//        else {
//            if (clawClose) {
//                robot.clawMotor.setPower(-1);
//            }
//            else {
//                if (clawRest) {
//                    robot.clawMotor.setPower(.00000001);
//                    //because the servo is being dumb
//                }
//            }
//        }




        //FIXME: commented out capstone
//        //CAPSTONE PLACE
//        capstoneOut = gamepad2.b;
//        capstoneIn = gamepad2.x;
//
//                if (capstoneOut && !capstoneIn) {
//                    robot.capstoneServo.setPower(-1);
//                } else {
//                    if (capstoneIn && !capstoneOut) {
//                        robot.capstoneServo.setPower(1);
//                    } else {
//                        robot.capstoneServo.setPower(0);
//                    }
                } //extra?
//            }

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
//            drive = -gamepad1.left_stick_y;
//            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
//            left  = drive + turn;
//            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
//            max = Math.max(Math.abs(left), Math.abs(right));
//            if (max > 1.0)
//            {
//                left /= max;
//                right /= max;
//            }

            // Output the safe vales to the motor drives.
//            robot.leftDrive.setPower(left);
//            robot.rightDrive.setPower(right);

            // Use gamepad left & right Bumpers to open and close the claw
//            if (gamepad1.right_bumper)
//                clawOffset += CLAW_SPEED;
//            else if (gamepad1.left_bumper)
//                clawOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
//            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
//            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)
//            if (gamepad1.y)
//                robot.leftArm.setPower(robot.ARM_UP_POWER);
//            else if (gamepad1.a)
//                robot.leftArm.setPower(robot.ARM_DOWN_POWER);
//            else
//                robot.leftArm.setPower(0.0);

            // Send telemetry message to signify robot running;
//            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
//            telemetry.addData("left",  "%.2f", left);
//            telemetry.addData("right", "%.2f", right);
//            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
//            sleep(50);
        //}
        //driving things
    }
        public void moveBot(double drive, double rotate, double strafe, double scaleFactor)
        {
            // This module takes inputs, normalizes them to DRIVE_SPEED, and drives the motors
//        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // How to normalize...Version 3
            //Put the raw wheel speeds into an array
            double wheelSpeeds[] = new double[4];
            wheelSpeeds[0] = drive + strafe - rotate;
            wheelSpeeds[1] = drive - strafe - rotate;
            wheelSpeeds[2] = drive - strafe + rotate;
            wheelSpeeds[3] = drive + strafe + rotate;
            // Find the magnitude of the first element in the array
            double maxMagnitude = Math.abs(wheelSpeeds[0]);
            // If any of the other wheel speeds are bigger, save that value in maxMagnitude
            for (int i = 1; i < wheelSpeeds.length; i++)
            {
                double magnitude = Math.abs(wheelSpeeds[i]);
                if (magnitude > maxMagnitude)
                {
                    maxMagnitude = magnitude;
                }
            }
            // Normalize all of the magnitudes to below 1
            if (maxMagnitude > 1.0)
            {
                for (int i = 0; i < wheelSpeeds.length; i++)
                {
                    wheelSpeeds[i] /= maxMagnitude;
                }
            }
            // Send the normalized values to the wheels, further scaled by the user
        robot.leftFrontDrive.setPower(scaleFactor * wheelSpeeds[0]);
        robot.leftRearDrive.setPower(scaleFactor * wheelSpeeds[1]);
        robot.rightFrontDrive.setPower(scaleFactor * wheelSpeeds[2]);
        robot.rightRearDrive.setPower(scaleFactor * wheelSpeeds[3]);








        }
}
