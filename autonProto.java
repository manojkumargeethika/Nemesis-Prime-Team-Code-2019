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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auton prototype", group="Minibot")
public class autonProto extends LinearOpMode {

    /* Declare OpMode members. Since we aren't using the pushbot hardware we simply delcare two DC motors*/
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo dropServo = null;
    private DcMotor intake = null;
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // we have Core Hex motors, creating a different count value
    static final double     DRIVE_GEAR_REDUCTION    = 0.75;     // we have no gears
    static final double     WHEEL_DIAMETER_INCHES   = 3.54331 ;     // we congverted the 90mm diameter to inches
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.14159265358979323);
    static final double     DRIVE_SPEED             = 0.5; // higher power = faster traversal
    static final double     TURN_SPEED              = 0.3; // higher power = faster traversal

    @Override
    public void runOpMode() {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get (DcMotor.class, "intake");
        dropServo = hardwareMap.get(Servo.class, "drop_Servo");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        dropServo.setPosition(1.0);
        waitForStart();
        dropServo.setPosition(0.6);
        sleep(500);
        VuforiaOrientator();
        encoderSequence("dumbDrive");


    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        if (opModeIsActive()) {
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (leftDrive.isBusy() && rightDrive.isBusy())) {
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            leftDrive.getCurrentPosition(),
                                            rightDrive.getCurrentPosition());
                telemetry.update();
            }
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void VuforiaOrientator() {
        //this code will make the robot turn until it sees a picture, once it sees a picture, it will use Vuforia to orient it perpendicular to a wall
        //and return a variable for the picture it is looking at.
    }
    public void encoderSequence(String robotSituation) {
        switch (robotSituation) {//this takes the information given by VuforiaOrientator to determine how it has to dead reckon to it's goals.
            case "dumbDrive":
                encoderDrive (DRIVE_SPEED, 28,28,10.0);
                intake.setPower(-1.0);
                sleep(4000);
                intake.setPower(0);
            case "dumbDriveDepotBlue":
                encoderDrive (DRIVE_SPEED, 28,28,10.0);
                intake.setPower(-1.0);
                sleep(4000);
                intake.setPower(0);
                encoderDrive (TURN_SPEED, 12, -12, 6);
                encoderDrive(DRIVE_SPEED, 144,144,10);
                break;
            case "dumbDriveCraterRed":
                encoderDrive(TURN_SPEED, -6,6,6);
                encoderDrive(DRIVE_SPEED, 10, 10, 5);
                encoderDrive(TURN_SPEED, -12,12,7);
                encoderDrive(DRIVE_SPEED, 38, 38, 8);
                intake.setPower(-1.0);
                sleep(4000);
                intake.setPower(0);
                encoderDrive (TURN_SPEED, -12, 12, 6);
                encoderDrive(DRIVE_SPEED, 144,144,10);
                break;
            case "dumbDriveDepotRed":
            encoderDrive (DRIVE_SPEED, 28,28,10.0);
            intake.setPower(-1.0);
            sleep(4000);
            intake.setPower(0);
            encoderDrive (TURN_SPEED, -12, 12, 6);
            encoderDrive(DRIVE_SPEED, 144,144,10);
            break;
            case "dumbDriveCraterBlue":
                encoderDrive(TURN_SPEED, -6,6,6);
                encoderDrive(DRIVE_SPEED, 10, 10, 5);
                encoderDrive(TURN_SPEED, -12,12,7);
                encoderDrive(DRIVE_SPEED, 38, 38, 8);
                intake.setPower(-1.0);
                sleep(4000);
                intake.setPower(0);
                encoderDrive (TURN_SPEED, -12, 12, 6);
                encoderDrive(DRIVE_SPEED, 144,144,10);
            default:
                telemetry.addData("you are subreme idiom and did not input a case",robotSituation);
                break;
        }
    }
}
