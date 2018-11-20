package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@TeleOp(name="dab", group ="Concept")
public class weather extends LinearOpMode {
    private static final String VUFORIA_KEY = "AcJHs6P/////AAABGbX6rkKBTExRkpf7wKGFLGJ4v2tqJ4hq+P26pYV5NgatcGWWR6oKdFhiULd3WSHHBHbMFxRgJT9bFf3GhqsuThI2vJetioOELtIly1pOKIt1PDZrDMFxWYkjDcSpw5cG3NS94vfIL3m8Nj0oiFC1hiFhks7D6+smLvc83OTmHfU0GDJwz3xJLMZ21OymD4pekzHwaEb6IJ4HnzarQjeFrBK5EDl0YCLgntdVesAwqel5psvdVl/dE8bkboNUsZ1e52GGPUYqEk5N6N7GhTCWO3YH9NVa4oKPa6yBw2jTQ/QVYQmc0kd7HyrjYxVaOo6xf9IggFHQkJ1BMOWVzq7rFK5rECsoYL793zAVIqXAtmud";

    VuforiaLocalizer vuforia;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    public void runOpMode() {


        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        parameters.cameraDirection   = CAMERA_CHOICE;
        parameters.vuforiaLicenseKey = VUFORIA_KEY ;

        OpenGLMatrix dab = OpenGLMatrix
                .translation(6,0,0);
        waitForStart();

        while (opModeIsActive()) {
            VectorF translation = dab.getTranslation();
            double TranslationX = translation.get(0);
            telemetry.addData("-->", TranslationX);
            telemetry.update();

        }


    }

}


