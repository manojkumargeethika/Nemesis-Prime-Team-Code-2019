package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name="Auton basic", group="Minibot")
public class autonBasic extends LinearOpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor elevatorDrive = null;
    private Servo markerServo = null;
    private ElapsedTime runtime = new ElapsedTime();
    String startQuadrant;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // we have Core Hex motors, creating a different count value
    static final double COUNTS_PER_MOTOR_REV_ELAVATOR = 288;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION_ELAVATOR = 72/30;     // This is < 1.0 if geared UP
    static final double DRIVE_GEAR_REDUCTION = 0.75;
    static final double ENCODER_MAX = 1563;
    static final double ELEVATION = 10.5;
    static final double COUNTS_PER_INCH_ELEVATOR = (ENCODER_MAX/ELEVATION);
    static final double WHEEL_DIAMETER_MM = 90;
    static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);
    static final double P_TURN_COEFF = 0.15;
    static final double HEADING_THRESHOLD = 1;
    static final double DRIVE_SPEED = 0.5; // higher power = faster traversal
    static final double TURN_SPEED = 0.3; // higher power = faster traversal
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        elevatorDrive = hardwareMap.get(DcMotor.class, "elevator_drive");
        markerServo = hardwareMap.get(Servo.class, "servo");
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // our right motor turns in the opposite direction so it has
        rightDrive.setDirection(DcMotor.Direction.REVERSE); // to be reversed to create forward motion when told to go forward
        elevatorDrive.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        runtime.reset();
        elevatorDrive (1, 10, 8);
        encoderDrive(1, 1466.8, 1466.8, 60);
        markerServo.setPosition(1);
        elevatorDrive(1,-10,20);
    }
    public void encoderDrive(double speed,
                             double leftMM, double rightMM,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() - (int) (leftMM * COUNTS_PER_MM);
            newRightTarget = rightDrive.getCurrentPosition() - (int) (rightMM * COUNTS_PER_MM);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors haveD finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void elevatorDrive(double speed, double elevatorDis, double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = elevatorDrive.getCurrentPosition() + (int) (elevatorDis * COUNTS_PER_INCH_ELEVATOR);
            elevatorDrive.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            elevatorDrive.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (elevatorDrive.isBusy())) {
                //if (digitalTouch.getState()) {
                //  elevatorDrive.setPower(0);
                //}
            }
            // Stop all motion;
            elevatorDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            elevatorDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
