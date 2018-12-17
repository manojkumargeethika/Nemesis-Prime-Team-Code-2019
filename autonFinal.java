package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

@Autonomous(name="Auton final", group="Minibot")
public class autonFinal extends LinearOpMode {
    public final String VUFORIA_KEY = "AcJHs6P/////AAABGbX6rkKBTExRkpf7wKGFLGJ4v2tqJ4hq+P26pYV5NgatcGWWR6oKdFhiULd3WSHHBHbMFxRgJT9bFf3GhqsuThI2vJetioOELtIly1pOKIt1PDZrDMFxWYkjDcSpw5cG3NS94vfIL3m8Nj0oiFC1hiFhks7D6+smLvc83OTmHfU0GDJwz3xJLMZ21OymD4pekzHwaEb6IJ4HnzarQjeFrBK5EDl0YCLgntdVesAwqel5psvdVl/dE8bkboNUsZ1e52GGPUYqEk5N6N7GhTCWO3YH9NVa4oKPa6yBw2jTQ/QVYQmc0kd7HyrjYxVaOo6xf9IggFHQkJ1BMOWVzq7rFK5rECsoYL793zAVIqXAtmud";

    /* Declare OpMode members. Since we aren't using the pushbot hardware we simply delcare two DC motors*/
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private BNO055IMU gyro;
    private Servo markerServo;
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
    static final double P_TURN_COEFF = 0.01;
    static final double HEADING_THRESHOLD = 0.25;
    static final double DRIVE_SPEED = 0.5; // higher power = faster traversal
    static final double TURN_SPEED = 0.3; // higher power = faster traversal
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

    public OpenGLMatrix lastLocation = null;
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
        // digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        leftDrive.setDirection(DcMotor.Direction.FORWARD); // our right motor turns in the opposite direction so it has
        rightDrive.setDirection(DcMotor.Direction.REVERSE); // to be reversed to create forward motion when told to go forward
        elevatorDrive.setDirection(DcMotor.Direction.REVERSE);
        // digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        OpenGLMatrix RedDepot = OpenGLMatrix
                .translation(1524, -1524, 0);
        OpenGLMatrix CraterRimRed = OpenGLMatrix
                .translation(-914, -1524, 0);
        OpenGLMatrix BlueDepot = OpenGLMatrix
                .translation(-1524, 1524, 0);
        OpenGLMatrix CraterRimBlue = OpenGLMatrix
                .translation(914, 1524, 0);
        // test whether this should be in inches or mm
        VectorF RedDepotT = RedDepot.getTranslation();
        VectorF BlueDepotT = BlueDepot.getTranslation();
        VectorF redCraterT = CraterRimRed.getTranslation();
        VectorF blueCraterT = CraterRimBlue.getTranslation();


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
            telemetry.addData("1", null);
            telemetry.update();
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.addData ("press", "play to start");
        telemetry.update();
        waitForStart();
        runtime.reset();
        //elevatorDrive(1, 10., 20);
        encoderDrive(DRIVE_SPEED, 100, 100, 10);
        //initVuforia();
       /*try
        {
            initVuforia();
        }
        catch (Exception e)‏
        {
            telemetry.addData("ERROR)", e);
        }
*/

        /** Wait for the game to begin */
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
                                telemetry.addData("2", null);
                                telemetry.update();
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
                                    gyroTurn(TURN_SPEED, 20);
                                    encoderDrive(DRIVE_SPEED, 30, 30, 5);

                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");

                                    gyroTurn(TURN_SPEED, -20);
                                    encoderDrive(DRIVE_SPEED, 30, 30, 5);
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");

                                    encoderDrive(DRIVE_SPEED, 30, 30, 5);
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
        targetVisible = false;
        //put elavator drop code here mannan you supreme idiom
        while (opModeIsActive() && !targetVisible) {
            double leftPower;
            double rightPower;

            // check all the trackable target to see which one (if any) is visible.
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
                break;
            }
            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                break;
            } else {
                telemetry.addData("Visible Target", "none");
                leftDrive.setPower(0.05);
                rightDrive.setPower(-0.05);
            }
            /*while (!targetVisible) {
                leftDrive.setPower(0.05);
                rightDrive.setPower(-0.05);
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    break;
                }
            }*/
            telemetry.update();
            break;
        }
        telemetry.addData("loop", "broken");
        telemetry.update();

        VectorF translation = lastLocation.getTranslation();
        VectorF translationblueRover = blueRoverLocationOnField.getTranslation();
        VectorF translationredFootprint = redFootprintLocationOnField.getTranslation();
        VectorF translationfrontCraters = frontCratersLocationOnField.getTranslation();
        VectorF translationbackSpace = backSpaceLocationOnField.getTranslation();
        double TranslationX = translation.get(0) / mmPerInch;
        double TranslationY = translation.get(1) / mmPerInch;
        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
        double Rotation = rotation.thirdAngle;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                switch (trackable.getName()) {
                    // this code relies on the robot turning right to see the picture to determine quadrant
                    case "blueRover":
                        double blueRoverDisX = translationblueRover.get(0) - translation.get(0);
                        double blueRoverDisY = translationblueRover.get(1) - translation.get(1);
                        double blueDepotDisX = BlueDepotT.get(0) - blueRoverDisX;
                        double blueDepotDisY = BlueDepotT.get(1) - blueRoverDisY;
                        gyroTurn(TURN_SPEED, Rotation);//facing exactly the picture
                        gyroTurn(TURN_SPEED, ((Math.atan(Math.abs(blueRoverDisY)) / Math.abs(blueRoverDisX)) * (180 / Math.PI)));//facing perpendicular to wall
                        gyroTurn(TURN_SPEED, -90);
                        gyro.initialize(parametersGyro);//sets it so that N on gameMap is 0 degrees
                        startQuadrant = "blueDepot";
                        break;
                    case "redFootprint":
                        double redFootprintDisX = translationredFootprint.get(0) - translation.get(0);
                        double redFootprintDisY = translationredFootprint.get(1) - translation.get(1);
                        double redDepotDisX = RedDepotT.get(0) - redFootprintDisX;
                        double redDepotDisY = RedDepotT.get(1) - redFootprintDisY;
                        gyroTurn(TURN_SPEED, Rotation);
                        gyroTurn(TURN_SPEED, (Math.atan(Math.abs(redFootprintDisY) / Math.abs(redFootprintDisX)) * (180 / Math.PI)));//facing perpendicular to wall
                        gyroTurn(TURN_SPEED, 90);
                        gyro.initialize(parametersGyro);
                        startQuadrant = "redDepot";
                        break;
                    case "frontCraters":
                        double frontCratersDisX = translationfrontCraters.get(0) - translation.get(0);
                        double frontCratersDisY = translationfrontCraters.get(1) - translation.get(1);
                        double blueCraterDisX = blueCraterT.get(0) - frontCratersDisX;
                        double blueCraterDisY = blueCraterT.get(1) - frontCratersDisY;
                        gyroTurn(TURN_SPEED, Rotation);
                        gyroTurn(TURN_SPEED, (Math.atan(Math.abs(frontCratersDisX) / Math.abs(frontCratersDisY)) * (180 / Math.PI)));//facing perpendicular to wall
                        gyro.initialize(parametersGyro);
                        startQuadrant = "blueCrater";
                        break;
                    case "backSpace":
                        double backSpaceDisX = translationbackSpace.get(0) - translation.get(0);
                        double backSpaceDisY = translationbackSpace.get(1) - translation.get(1);
                        double redCraterDisX = redCraterT.get(0) - backSpaceDisX;
                        double redCraterDisY = redCraterT.get(1) - backSpaceDisY;
                        gyroTurn(TURN_SPEED, Rotation);
                        gyroTurn(TURN_SPEED, (Math.atan(Math.abs(backSpaceDisX) / Math.abs(backSpaceDisY)) * (180 / Math.PI)));//facing perpendicular to wall
                        gyroTurn(TURN_SPEED, 180);
                        gyro.initialize(parametersGyro);
                        startQuadrant = "redCrater";
                        break;
                    default:
                        break;
                }
            }
        }

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                double redCraterDisX = redCraterT.get(0) - translation.get(0);
                double redCraterDisY = redCraterT.get(1) - translation.get(0);
                double blueCraterDisX = blueCraterT.get(0) - translation.get(0);
                double blueCraterDisY = blueCraterT.get(1) - translation.get(1);
                double blueDepotDisX = BlueDepotT.get(0) - translation.get(0);
                double blueDepotDisY = BlueDepotT.get(1) - translation.get(1);
                double redDepotDisX = RedDepotT.get(0) - translation.get(0);
                double redDepotDisY = RedDepotT.get(1) - translation.get(1);
                switch (trackable.getName()) {
                    // this code relies on the robot turning right to see the picture to determine quadrant
                    case "blueRover":
                        switch (startQuadrant) {
                            case "blueDepot":
                                gyroTurnTo(TURN_SPEED, 90);
                                encoderDrive(DRIVE_SPEED, Math.abs(blueDepotDisX), Math.abs(blueDepotDisX), 20);
                                gyroTurnTo(TURN_SPEED, 0);
                                encoderDrive(DRIVE_SPEED, Math.abs(blueDepotDisY), Math.abs(blueDepotDisY), 20);
                                markerServo.setPosition(1);
                                sleep(1500);
                                gyroTurnTo(TURN_SPEED, 180);
                                encoderDrive(DRIVE_SPEED, Math.abs(blueDepotDisY+blueCraterDisY), Math.abs(blueDepotDisY+blueCraterDisY), 10);
                                break;
                            case "blueCrater":
                                gyroTurnTo(TURN_SPEED, 90);
                                encoderDrive(DRIVE_SPEED, Math.abs(blueDepotDisX), Math.abs(blueDepotDisX), 20);
                                gyroTurnTo(TURN_SPEED, 0);
                                encoderDrive(DRIVE_SPEED, Math.abs(blueDepotDisY), Math.abs(blueDepotDisY), 20);
                                markerServo.setPosition(1);
                                sleep(1500);
                                gyroTurnTo(TURN_SPEED, 180);
                                encoderDrive(DRIVE_SPEED, Math.abs(blueDepotDisY+blueCraterDisY), Math.abs(blueDepotDisY+blueCraterDisY), 10);
                                break;
                            default:
                                telemetry.addData("what did you do xd", 0);
                                telemetry.update();
                                break;
                        }
                        break;
                    case "redFootprint":
                        switch (startQuadrant) {
                            case "redDepot":
                                gyroTurnTo(TURN_SPEED, 270);
                                encoderDrive(DRIVE_SPEED, Math.abs(redDepotDisX), Math.abs(redDepotDisX), 20);
                                gyroTurnTo(TURN_SPEED, 180);
                                encoderDrive(DRIVE_SPEED, Math.abs(redDepotDisY), Math.abs(redDepotDisY), 20);
                                markerServo.setPosition(1);
                                sleep(1500);
                                gyroTurnTo(TURN_SPEED, 359.99999);
                                encoderDrive(DRIVE_SPEED, Math.abs(redDepotDisY+redCraterDisY), Math.abs(redDepotDisY+redCraterDisY), 10);
                                break;
                            case "redCrater":
                                gyroTurnTo(TURN_SPEED, 270);
                                encoderDrive(DRIVE_SPEED, Math.abs(redDepotDisX), Math.abs(redDepotDisX), 20);
                                gyroTurnTo(TURN_SPEED, 180);
                                encoderDrive(DRIVE_SPEED, Math.abs(redDepotDisY), Math.abs(redDepotDisY), 20);
                                markerServo.setPosition(1);
                                sleep(1500);
                                gyroTurnTo(TURN_SPEED, 359.99999);
                                encoderDrive(DRIVE_SPEED, Math.abs(redDepotDisY+redCraterDisY), Math.abs(redDepotDisY+redCraterDisY), 10);
                                break;
                            default:
                                telemetry.addData("what did you do xd", 0);
                                telemetry.update();
                                break;
                        }
                        break;
                    case "frontCraters":
                        switch (startQuadrant) {
                            case "redDepot":
                                gyroTurnTo(TURN_SPEED, 180);
                                encoderDrive(DRIVE_SPEED, Math.abs(redDepotDisY), Math.abs(redDepotDisY), 20);
                                gyroTurnTo(TURN_SPEED, 270);
                                encoderDrive(DRIVE_SPEED, Math.abs(redDepotDisX), Math.abs(redDepotDisX), 20);
                                markerServo.setPosition(1);
                                sleep(3000);
                                gyroTurn(TURN_SPEED, 360);
                                encoderDrive (TURN_SPEED, Math.abs(redDepotDisY + redCraterDisY), Math.abs(redDepotDisY + redCraterDisY), 20);
                                break;
                            case "blueCrater":
                                gyroTurnTo(TURN_SPEED, 90);
                                encoderDrive(DRIVE_SPEED, Math.abs(blueDepotDisX), Math.abs(blueDepotDisX), 20);
                                gyroTurnTo(TURN_SPEED, 360);
                                encoderDrive(DRIVE_SPEED, Math.abs(blueDepotDisX), Math.abs(blueDepotDisX), 20);
                                markerServo.setPosition(1);
                                sleep(3000);
                                encoderDrive (TURN_SPEED, -Math.abs(blueDepotDisY + blueCraterDisY), -Math.abs(blueDepotDisY + blueCraterDisY), 20);
                                break;
                            default:
                                telemetry.addData("what did you do xd", 0);
                                telemetry.update();
                                break;
                        }
                        break;
                    case "backSpace":
                        switch (startQuadrant) {
                            case "blueDepot":
                                gyroTurnTo(TURN_SPEED, 90);
                                encoderDrive(DRIVE_SPEED, Math.abs(blueDepotDisX), Math.abs(blueDepotDisX), 20);
                                gyroTurnTo(TURN_SPEED, 360);
                                encoderDrive(DRIVE_SPEED, Math.abs(blueDepotDisX), Math.abs(blueDepotDisX), 20);
                                markerServo.setPosition(1);
                                sleep(3000);
                                encoderDrive (TURN_SPEED, -Math.abs(blueDepotDisY + blueCraterDisY), -Math.abs(blueDepotDisY + blueCraterDisY), 20);
                                break;
                            case "redCrater":
                                gyroTurnTo(TURN_SPEED, 180);
                                encoderDrive(DRIVE_SPEED, Math.abs(redDepotDisY), Math.abs(redDepotDisY), 20);
                                gyroTurnTo(TURN_SPEED, 270);
                                encoderDrive(DRIVE_SPEED, Math.abs(redDepotDisX), Math.abs(redDepotDisX), 20);
                                markerServo.setPosition(1);
                                sleep(3000);
                                gyroTurn(TURN_SPEED, 360);
                                encoderDrive (TURN_SPEED, Math.abs(redDepotDisY + redCraterDisY), Math.abs(redDepotDisY + redCraterDisY), 20);
                                break;
                            default:
                                telemetry.addData("what did you do xd", 0);
                                telemetry.update();
                                break;
                        }
                        break;
                    default:
                        telemetry.addData("what did you do xd", 0);
                        telemetry.update();
                        break;
                }
            }
        }
        while (opModeIsActive()) {
            double leftPower;
            double rightPower;

            // check all the trackable target to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }
        while (!targetVisible) {
            leftDrive.setPower(TURN_SPEED);
            rightDrive.setPower(-TURN_SPEED);
            if (targetVisible) {
                break;
            }
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
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
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
