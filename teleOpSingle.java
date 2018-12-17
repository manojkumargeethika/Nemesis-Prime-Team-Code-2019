package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="roverRuckusTeleopSingle", group="Linear Opmode")
public class teleOpSingle extends LinearOpMode {
    public org.firstinspires.ftc.teamcode.subsystems.driveTrain dTrain = new org.firstinspires.ftc.teamcode.subsystems.driveTrain();
    public org.firstinspires.ftc.teamcode.subsystems.intake intake = new org.firstinspires.ftc.teamcode.subsystems.intake();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor elevatorDrive = null;
    private Servo servo = null;
    static final double COUNTS_PER_MOTOR_REV_ELAVATOR = 288;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION_ELAVATOR = 72/30;     // This is < 1.0 if geared UP
    static final double SPROCKET_DIAMETER = 1;     // For figuring circumference
    static final double COUNTS_PER_INCH_ELEVATOR = (COUNTS_PER_MOTOR_REV_ELAVATOR * DRIVE_GEAR_REDUCTION_ELAVATOR) /
            (SPROCKET_DIAMETER * 3.14159265358979323);
    static final double ELAVATOR_SENSETIVITY = 0.1;
    private ElapsedTime runtime = new ElapsedTime();
    //private DigitalChannel digitalTouch = null;
    @Override
    public void runOpMode() {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        elevatorDrive = hardwareMap.get(DcMotor.class, "elevator_drive");
        servo = hardwareMap.get(Servo.class, "servo");
      //  digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        elevatorDrive.setDirection(DcMotor.Direction.FORWARD);
        elevatorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        runtime.reset();
        double speedlimiter = 0;
        while (opModeIsActive()) {
            if (gamepad1.y) {
                speedlimiter = 1.0;
            } else {
                speedlimiter = 0.5;
            }
            if (gamepad1.a) {
                elevatorDrive.setPower(1);
            } else if (gamepad1.b) {
                elevatorDrive.setPower(-1);
            } else {
                elevatorDrive.setPower(0);
            }
            teleopInput(gamepad1.left_stick_y,gamepad1.right_stick_x,speedlimiter, leftDrive, rightDrive);
            telemetry.addData("elevatorPosition %7d", elevatorDrive.getCurrentPosition());
            telemetry.addData("servoPosition %7d", servo.getPosition());
            telemetry.update();
        }
    }
    public void teleopInput(double drive, double turn, double speedLimiter, DcMotor leftDrive, DcMotor rightDrive) {
        double leftPower = 0;
        double rightPower = 0;
        leftPower = Range.clip(drive + turn, -speedLimiter, speedLimiter);
        rightPower = Range.clip(drive - turn, -speedLimiter, speedLimiter);
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
    public void elavatorMove(double speed, double inches, double timeoutS) {
        int newTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = elevatorDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_ELEVATOR);
            elevatorDrive.setTargetPosition(newTarget);


            // Turn On RUN_TO_POSITION

            elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorDrive.setPower(Math.abs(1));

            // reset the timeout time and start motion.
            runtime.reset();


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (elevatorDrive.isBusy())) {
               // if(digitalTouch.getState()){
                 //   elevatorDrive.setPower(0);
                //}
                telemetry.addData("Path1", "Running to %7d,", newTarget);
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            // Stop all motion;
            elevatorDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            elevatorDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
}
