package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Auton less basic crater", group="Minibot")
public class autonLessBasicCrater extends LinearOpMode {
   // DigitalChannel digitalTouch;  // Hardware Device Object
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor elevatorDrive = null;
    private Servo markerServo = null;
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU gyro;
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
    static final double P_TURN_COEFF = 0.01;
    static final double HEADING_THRESHOLD = 0.25;
    static final double DRIVE_SPEED = 0.5; // higher power = faster traversal
    static final double TURN_SPEED = 0.3; // higher power = faster traversal
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable


    public void runOpMode() {
        // get a reference to our digitalTouch object.
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        BNO055IMU.Parameters parametersGyro = new BNO055IMU.Parameters();
        parametersGyro.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersGyro.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersGyro.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersGyro.loggingEnabled = true;
        parametersGyro.loggingTag = "IMU";
        parametersGyro.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        elevatorDrive = hardwareMap.get(DcMotor.class, "elevator_drive");
        markerServo = hardwareMap.get(Servo.class, "servo");
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // our right motor turns in the opposite direction so it has
        rightDrive.setDirection(DcMotor.Direction.REVERSE); // to be reversed to create forward motion when told to go forward
        elevatorDrive.setDirection(DcMotor.Direction.REVERSE);
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        waitForStart();
        telemetry.addData ("heading:", getHeading());
        telemetry.update();
        elevatorDrive (1, 10, 8);
        gyro.initialize(parametersGyro);
        encoderDrive(DRIVE_SPEED, 200, 200, 5); //drive forward
        gyroTurnTo(TURN_SPEED, 225); //turning to depo
        encoderDrive(DRIVE_SPEED, 1085, 1085, 5); //driving to depot
        gyroTurnTo (TURN_SPEED, 315); //turning to crater
        encoderDrive(DRIVE_SPEED, 1500, 1500, 10); //driving to crater
        markerServo.setPosition(1);
        sleep(1500);
        encoderDrive(DRIVE_SPEED,  -2146.4, -2146.4, 10);
        elevatorDrive(1,-10,20);
        //gyroTurnTo(1, 315);
        //encoderDrive(1,2203.2, 2203.2, 10);

       /* if (digitalTouch.getState() == true) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
        }*/

        telemetry.update();
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
    public void gyroTurnTo(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        double angleTarget;
        angleTarget = getHeading() - angle;

        while (opModeIsActive() && !onHeading(speed, angleTarget, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = -(speed * steer);
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        telemetry.addData("heading", "%5.2f:%5.2f%5.2f", getHeading(),getRoll(),getPitch());

        return onTarget;
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        //while (robotError >= 180) robotError -= 360;
        //while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public double getHeading() {
        Orientation angles = gyro.getAngularOrientation();
        return angles.firstAngle+180;
    }
    public double getRoll() {
        Orientation angles = gyro.getAngularOrientation();
        return angles.secondAngle;
    }
    public double getPitch() {
        Orientation angles = gyro.getAngularOrientation();
        return angles.thirdAngle;
    }
}