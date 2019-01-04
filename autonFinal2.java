
        package org.firstinspires.ftc.teamcode;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcore.external.ClassFactory;
        import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
        import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
        import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
        import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

        import java.util.ArrayList;
        import java.util.List;

        import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
        import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
// clockwise is negative
/*
Next steps:
-
Bugs:
Suggestions:
Notes for EN:
- Turning doesn't work, too blurry
 */
@Autonomous(name="Autonfinal", group="Minibot")
public class autonFinal2 extends LinearOpMode {
    public final String VUFORIA_KEY = "AcWJXfv/////AAABmXTvMEsBOEYkiGqMsGZyklUBGfk5cSsLyBZx0YTUz4txj9n9lF3yHQWwQIFc+gC2pdWkKk3iWgbSza68dp0T2zqc+1sG6S5G6VAXxnNBUsH6rjRa+6p1kPIsiEQdezRl4m5VeATR5SzGECvIbIhtc3nWjjquoM/d8+R7QCMOrAPRwf9bhK6Ah2tgIuPkwVwkp+G1q4/qaFrLMGcDxRDlwNTFMZfnmTLt18Xe6Q54eDPb6Bw2MHmfUXSXbeqiahjFGQz40bh0TH3vAp47L88U3oXSo+YgV2TyT1PhQE7SUE71ucVNQ9PUM0OfIKFTtHy2MaXsE2dOsj+WAJG5J3iGO530Gaod7DtKZSCPC02SyADM";

    /* Declare OpMode members. Since we aren't using the pushbot hardware we simply delcare two DC motors*/
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private BNO055IMU gyro;
    private Servo markerServo;
    private ElapsedTime runtime = new ElapsedTime();
    String startQuadrant;
    String trackableName;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // we have Core Hex motors, creating a different count value
    static final double COUNTS_PER_MOTOR_REV_ELAVATOR = 288;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION_ELAVATOR = 72 / 30;     // This is < 1.0 if geared UP
    static final double DRIVE_GEAR_REDUCTION = 0.75;
    static final double ENCODER_MAX = 1563;
    static final double ELEVATION = 10.5;
    static final double COUNTS_PER_INCH_ELEVATOR = (ENCODER_MAX / ELEVATION);
    static final double WHEEL_DIAMETER_MM = 90;
    static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);
    static final double P_TURN_COEFF = 0.01;
    static final double HEADING_THRESHOLD = 1;
    static final double DRIVE_SPEED = 0.5; // higher power = faster traversal
    static final double TURN_SPEED = 0.2; // higher power = faster traversal
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    static final double CHAIN_LENGTH = 0.05;
    private TFObjectDetector tfod;
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    static double i_position = 1.0; //initial position for drop servo
    static double f_position = 0.6; //final position
    public Servo lockServo;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    OpenGLMatrix redDepot = OpenGLMatrix
            .translation(1524, -1524, 0);
    OpenGLMatrix redCrater = OpenGLMatrix
            .translation(-914, -1524, 0);
    OpenGLMatrix blueDepot = OpenGLMatrix
            .translation(-1524, 1524, 0);
    OpenGLMatrix blueCrater = OpenGLMatrix
            .translation(914, 1524, 0);
    VectorF redDepotT = redDepot.getTranslation();
    VectorF blueDepotT = blueDepot.getTranslation();
    VectorF redCraterT = redCrater.getTranslation();
    VectorF blueCraterT = blueCrater.getTranslation();
    double secondDrivex;
    double secondDrivey;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch = 25.4f;
    public static final float mmFTCFieldWidth = (72) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    public static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private DcMotor elevatorDrive = null;
    // private DigitalChannel digitalTouch = null;

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    public OpenGLMatrix lastLocation;
    private boolean targetVisible = false;
    VuforiaLocalizer vuforia;

    public void runOpMode() {
        BNO055IMU.Parameters parametersGyro = new BNO055IMU.Parameters();
        parametersGyro.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersGyro.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersGyro.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersGyro.loggingEnabled = true;
        parametersGyro.loggingTag = "IMU";
        parametersGyro.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        markerServo = hardwareMap.get(Servo.class, "servo");
        gyro.initialize(parametersGyro);

        elevatorDrive = hardwareMap.get(DcMotor.class, "elevator_drive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD); // our right motor turns in the opposite direction so it has
        rightDrive.setDirection(DcMotor.Direction.REVERSE); // to be reversed to create forward motion when told to go forward
        elevatorDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersVuforia = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parametersVuforia.vuforiaLicenseKey = VUFORIA_KEY;
        parametersVuforia.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parametersVuforia);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT = 50;   // eg: Camera is 30 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 235;   // eg: Camera is 130 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 67;     // eg: Camera is 50 mm to the right of robot center
        // adjust

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        0, -90, 0));
        // adjust

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parametersVuforia.cameraDirection);
        }
        targetsRoverRuckus.activate();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.addLine("Press play to start.");
        telemetry.update();
        waitForStart();
        runtime.reset();
        //elevatorDrive(1, 10., 20);
        encoderDrive(DRIVE_SPEED, 254, 254, 10);

        //put elavator drop code here mannan you supreme idiom
        double leftPower;
        double rightPower;
        targetVisible = false;
        while (lastLocation == null) {
            leftDrive.setPower(0.1);
            rightDrive.setPower(-0.1);
           // sleep(250);
            for (VuforiaTrackable trackable : allTrackables) {

                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                        trackableName = trackable.getName();
                        telemetry.update();
                    }
                }
            }
            break;
        }
        // express position (translation) of robot in inches.
        VectorF translation = lastLocation.getTranslation();
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

        // express the rotation of the robot in degrees.
        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        telemetry.update();
        VectorF translationblueRover = blueRoverLocationOnField.getTranslation();
        VectorF translationredFootprint = redFootprintLocationOnField.getTranslation();
        VectorF translationfrontCraters = frontCratersLocationOnField.getTranslation();
        VectorF translationbackSpace = backSpaceLocationOnField.getTranslation();
        double Rotation = rotation.thirdAngle;
        switch (trackableName) {
            // this code relies on the robot turning right to see the picture to determine quadrant
            case "Blue-Rover":
                double blueRoverDisX = translationblueRover.get(0) - translation.get(0);
                double blueRoverDisY = translationblueRover.get(1) - translation.get(1);
                gyroTurn(TURN_SPEED, ((Math.atan(Math.abs(blueRoverDisY)) / Math.abs(blueRoverDisX)) * (180 / Math.PI)));//facing perpendicular to wall
                sleep(1000);
                gyroTurn(TURN_SPEED, 45);
                sleep(1000);
                gyro.initialize(parametersGyro);//sets it so that facing the minerals on gameMap is 0 degrees
                startQuadrant = "blueDepot";
                tfod("blueDepot", translation.get(0), translation.get(1));
                double disb = Math.hypot(secondDrivex, secondDrivey);
                double finaldis = Math.abs(blueCraterT.get(0) - blueDepotT.get(0));
                gyroTurn(TURN_SPEED, -60);
                encoderDrive(DRIVE_SPEED, disb, disb, 60); // bring us to depot
                markerServo.setPosition(1);
                gyroTurn(TURN_SPEED, -100);
                encoderDrive(DRIVE_SPEED, finaldis, finaldis, 60); // bring us to crater
                break;
            case "Red-Footprint":
                double redFootprintDisX = translationredFootprint.get(0) - translation.get(0);
                double redFootprintDisY = translationredFootprint.get(1) - translation.get(1);
                gyroTurn(TURN_SPEED, Rotation);
                gyroTurn(TURN_SPEED, (Math.atan(Math.abs(redFootprintDisY) / Math.abs(redFootprintDisX)) * (180 / Math.PI)));//facing perpendicular to wall
                gyroTurn(TURN_SPEED, 45);
                gyro.initialize(parametersGyro);// set it so facing minerals on gamemap is 0 degrees
                startQuadrant = "redDepot";
                tfod("redDepot", translation.get(0), translation.get(1));
                double disb2 = Math.hypot(secondDrivex, secondDrivey);
                double finaldis2 = Math.abs(blueCraterT.get(0) - blueDepotT.get(0));
                gyroTurn(TURN_SPEED, -60);
                encoderDrive(DRIVE_SPEED, disb2, disb2, 60); // bring us to depot
                markerServo.setPosition(1);
                gyroTurn(TURN_SPEED, -100);
                encoderDrive(DRIVE_SPEED, finaldis2, finaldis2, 60); // bring us to crater
                break;
            case "Front-Craters":
                double frontCratersDisX = translationfrontCraters.get(0) - translation.get(0);
                double frontCratersDisY = translationfrontCraters.get(1) - translation.get(1);
                gyroTurn(TURN_SPEED, Rotation);
                gyroTurn(TURN_SPEED, (Math.atan(Math.abs(frontCratersDisX) / Math.abs(frontCratersDisY)) * (180 / Math.PI)));//facing perpendicular to wall
                gyroTurn(TURN_SPEED, 45);
                gyro.initialize(parametersGyro); // set it so facing minerals on gamemap is 0 degrees
                startQuadrant = "blueCrater";
                tfod("blueCrater", translation.get(0), translation.get(1));
                double disb3 = Math.hypot(secondDrivex, secondDrivey);
                double finaldis3 = Math.abs(blueCraterT.get(0) - blueDepotT.get(0));
                gyroTurn(TURN_SPEED, -60);
                encoderDrive(DRIVE_SPEED, disb3, disb3, 60); // bring us to depot
                markerServo.setPosition(1);
                gyroTurn(TURN_SPEED, -100);
                encoderDrive(DRIVE_SPEED, finaldis3, finaldis3, 60); // bring us to crater
                break;
            case "Back-Space":
                double backSpaceDisX = translationbackSpace.get(0) - translation.get(0);
                double backSpaceDisY = translationbackSpace.get(1) - translation.get(1);
                gyroTurn(TURN_SPEED, Rotation);
                gyroTurn(TURN_SPEED, (Math.atan(Math.abs(backSpaceDisX) / Math.abs(backSpaceDisY)) * (180 / Math.PI)));//facing perpendicular to wall
                gyroTurn(TURN_SPEED, 225);
                gyro.initialize(parametersGyro); // set it so facing minerals on gamemap is 0 degrees
                startQuadrant = "redCrater";
                tfod("redCrater", translation.get(0), translation.get(1));
                double disb4 = Math.hypot(secondDrivex, secondDrivey);
                double finaldis4 = Math.abs(blueCraterT.get(0) - blueDepotT.get(0));
                gyroTurn(TURN_SPEED, -60);
                encoderDrive(DRIVE_SPEED, disb4, disb4, 60); // bring us to depot
                markerServo.setPosition(1);
                gyroTurn(TURN_SPEED, -100);
                encoderDrive(DRIVE_SPEED, finaldis4, finaldis4, 60); // bring us to crater
                break;
            default:
                break;
        }
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

    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        double angleTarget;
        Orientation angles = gyro.getAngularOrientation();
        angleTarget = angles.firstAngle + angle;

        while (opModeIsActive() && !onHeading(speed, angleTarget, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    public void gyroTurnTo(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
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
        telemetry.addData("heading", "%5.1f,", getHeading());

        return onTarget;
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    /**
     * getError determines the error between the ta/rget angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;
        Orientation angles = gyro.getAngularOrientation();

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        //while (robotError > 180) robotError -= 360;
        //while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public double getHeading() {
        Orientation angles = gyro.getAngularOrientation();
        return angles.firstAngle + 180;
    }
    public void tfod(String startQuadrant, double x, double y) {
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() <= 10) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    // code goes here
                                    switch (startQuadrant) {
                                        case "redDepot":
                                            OpenGLMatrix left1 = OpenGLMatrix.translation(1219, -609, 0);
                                            VectorF left1T = left1.getTranslation();
                                            double drivetoMineralx = Math.abs(x - left1T.get(0));
                                            double drivetoMineraly = Math.abs(y - left1T.get(1));
                                            double dis = Math.hypot(drivetoMineralx, drivetoMineraly);
                                            gyroTurn(0.3, 30);
                                            encoderDrive(DRIVE_SPEED, dis, dis, 60);
                                            secondDrivex = Math.abs(redDepotT.get(0) - drivetoMineralx);
                                            secondDrivey = Math.abs(redDepotT.get(1) - drivetoMineraly);
                                            break;
                                        case "redCrater":
                                            OpenGLMatrix left2 = OpenGLMatrix.translation(-609, -1219, 0);
                                            VectorF left2T = left2.getTranslation();
                                            double drivetoMineralx2 = Math.abs(x - left2T.get(0));
                                            double drivetoMineraly2 = Math.abs(y - left2T.get(1));
                                            double dis2 = Math.hypot(drivetoMineralx2, drivetoMineraly2);
                                            gyroTurn(0.3, 30);
                                            encoderDrive(DRIVE_SPEED, dis2, dis2,60);
                                            secondDrivex = Math.abs(redDepotT.get(0) - drivetoMineralx2);
                                            secondDrivey = Math.abs(redDepotT.get(1) - drivetoMineraly2);
                                            break;
                                        case "blueDepot":
                                            OpenGLMatrix left3 = OpenGLMatrix.translation(-1219, 609, 0);
                                            VectorF left3T = left3.getTranslation();
                                            double drivetoMineralx3 = Math.abs(x - left3T.get(0));
                                            double drivetoMineraly3 = Math.abs(y - left3T.get(1));
                                            double dis3 = Math.hypot(drivetoMineralx3, drivetoMineraly3);
                                            gyroTurn(0.3, 30);
                                            encoderDrive(DRIVE_SPEED, dis3, dis3,60);
                                            secondDrivex = Math.abs(redDepotT.get(0) - drivetoMineralx3);
                                            secondDrivey = Math.abs(redDepotT.get(1) - drivetoMineraly3);
                                            break;
                                        case "blueCrater":
                                            OpenGLMatrix left4 = OpenGLMatrix.translation(609, 1219, 0);
                                            VectorF left4T = left4.getTranslation();
                                            double drivetoMineralx4 = Math.abs(x - left4T.get(0));
                                            double drivetoMineraly4 = Math.abs(y - left4T.get(1));
                                            double dis4 = Math.hypot(drivetoMineralx4, drivetoMineraly4);
                                            gyroTurn(0.3, 30);
                                            encoderDrive(DRIVE_SPEED, dis4, dis4, 60);
                                            secondDrivex = Math.abs(redDepotT.get(0) - drivetoMineralx4);
                                            secondDrivey = Math.abs(redDepotT.get(1) - drivetoMineraly4);
                                            break;
                                    }
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    // code goes here
                                    switch (startQuadrant) {
                                        case "redDepot":
                                            OpenGLMatrix right1 = OpenGLMatrix.translation(609, -1219, 0);
                                            VectorF right1T = right1.getTranslation();
                                            double drivetoMineralx = Math.abs(x - right1T.get(0));
                                            double drivetoMineraly = Math.abs(y - right1T.get(1));
                                            double dis = Math.hypot(drivetoMineralx, drivetoMineraly);
                                            gyroTurn(0.3, -30);
                                            encoderDrive(DRIVE_SPEED, dis, dis, 60);
                                            secondDrivex = Math.abs(redDepotT.get(0) - drivetoMineralx);
                                            secondDrivey = Math.abs(redDepotT.get(1) - drivetoMineraly);
                                            break;
                                        case "redCrater":
                                            OpenGLMatrix right2 = OpenGLMatrix.translation(-1219, -609, 0);
                                            VectorF right2T = right2.getTranslation();
                                            double drivetoMineralx2 = Math.abs(x - right2T.get(0));
                                            double drivetoMineraly2 = Math.abs(y - right2T.get(1));
                                            double dis2 = Math.hypot(drivetoMineralx2, drivetoMineraly2);
                                            gyroTurn(0.3, -30);
                                            encoderDrive(DRIVE_SPEED, dis2, dis2, 60);
                                            secondDrivex = Math.abs(redDepotT.get(0) - drivetoMineralx2);
                                            secondDrivey = Math.abs(redDepotT.get(1) - drivetoMineraly2);
                                            break;
                                        case "blueDepot":
                                            OpenGLMatrix right3 = OpenGLMatrix.translation(-609, 1219, 0);
                                            VectorF right3T = right3.getTranslation();
                                            double drivetoMineralx3 = Math.abs(x - right3T.get(0));
                                            double drivetoMineraly3 = Math.abs(y - right3T.get(1));
                                            double dis3 = Math.hypot(drivetoMineralx3, drivetoMineraly3);
                                            gyroTurn(0.3, -30);
                                            encoderDrive(DRIVE_SPEED, dis3, dis3, 60);
                                            secondDrivex = Math.abs(redDepotT.get(0) - drivetoMineralx3);
                                            secondDrivey = Math.abs(redDepotT.get(1) - drivetoMineraly3);
                                            break;
                                        case "blueCrater":
                                            OpenGLMatrix right4 = OpenGLMatrix.translation(1219, 609, 0);
                                            VectorF right4T = right4.getTranslation();
                                            double drivetoMineralx4 = Math.abs(x - right4T.get(0));
                                            double drivetoMineraly4 = Math.abs(y - right4T.get(1));
                                            double dis4 = Math.hypot(drivetoMineralx4, drivetoMineraly4);
                                            gyroTurn(0.3, -30);
                                            encoderDrive(DRIVE_SPEED, dis4, dis4, 60);
                                            secondDrivex = Math.abs(redDepotT.get(0) - drivetoMineralx4);
                                            secondDrivey = Math.abs(redDepotT.get(1) - drivetoMineraly4);
                                            break;
                                    }
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    // code goes here
                                    switch (startQuadrant) {
                                        case "redDepot":
                                            OpenGLMatrix center1 = OpenGLMatrix.translation(914, -914, 0);
                                            VectorF center1T = center1.getTranslation();
                                            double drivetoMineralx = Math.abs(x - center1T.get(0));
                                            double drivetoMineraly = Math.abs(y - center1T.get(1));
                                            double dis = Math.hypot(drivetoMineralx, drivetoMineraly);
                                            encoderDrive(DRIVE_SPEED, dis, dis, 60);
                                            secondDrivex = Math.abs(redDepotT.get(0) - drivetoMineralx);
                                            secondDrivey = Math.abs(redDepotT.get(1) - drivetoMineraly);
                                            break;
                                        case "redCrater":
                                            OpenGLMatrix center2 = OpenGLMatrix.translation(-914, -914, 0);
                                            VectorF center2T = center2.getTranslation();
                                            double drivetoMineralx2 = Math.abs(x - center2T.get(0));
                                            double drivetoMineraly2 = Math.abs(y - center2T.get(1));
                                            double dis2 = Math.hypot(drivetoMineralx2, drivetoMineraly2);
                                            encoderDrive(DRIVE_SPEED, dis2, dis2, 60);
                                            secondDrivex = Math.abs(redDepotT.get(0) - drivetoMineralx2);
                                            secondDrivey = Math.abs(redDepotT.get(1) - drivetoMineraly2);
                                            break;
                                        case "blueDepot":
                                            OpenGLMatrix center3 = OpenGLMatrix.translation(-914, 914, 0);
                                            VectorF center3T = center3.getTranslation();
                                            double drivetoMineralx3 = Math.abs(x - center3T.get(0));
                                            double drivetoMineraly3 = Math.abs(y - center3T.get(1));
                                            double dis3 = Math.hypot(drivetoMineralx3, drivetoMineraly3);
                                            encoderDrive(DRIVE_SPEED, dis3, dis3, 60);
                                            secondDrivex = Math.abs(redDepotT.get(0) - drivetoMineralx3);
                                            secondDrivey = Math.abs(redDepotT.get(1) - drivetoMineraly3);
                                            break;
                                        case "blueCrater":
                                            OpenGLMatrix center4 = OpenGLMatrix.translation(914, 914, 0);
                                            VectorF center4T = center4.getTranslation();
                                            double drivetoMineralx4 = Math.abs(x - center4T.get(0));
                                            double drivetoMineraly4 = Math.abs(y - center4T.get(1));
                                            double dis4 = Math.hypot(drivetoMineralx4, drivetoMineraly4);
                                            encoderDrive(DRIVE_SPEED, dis4, dis4, 60);
                                            secondDrivex = Math.abs(redDepotT.get(0) - drivetoMineralx4);
                                            secondDrivey = Math.abs(redDepotT.get(1) - drivetoMineraly4);
                                            break;

                                    }
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
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


