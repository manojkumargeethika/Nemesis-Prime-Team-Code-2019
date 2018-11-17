package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Vuforia;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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

@TeleOp(name="dab", group ="Concept")
public class weather extends LinearOpMode {

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;
    private static final float mmTargetHeight   = (6) * mmPerInch;
    private static final String VUFORIA_KEY = "AcJHs6P/////AAABGbX6rkKBTExRkpf7wKGFLGJ4v2tqJ4hq+P26pYV5NgatcGWWR6oKdFhiULd3WSHHBHbMFxRgJT9bFf3GhqsuThI2vJetioOELtIly1pOKIt1PDZrDMFxWYkjDcSpw5cG3NS94vfIL3m8Nj0oiFC1hiFhks7D6+smLvc83OTmHfU0GDJwz3xJLMZ21OymD4pekzHwaEb6IJ4HnzarQjeFrBK5EDl0YCLgntdVesAwqel5psvdVl/dE8bkboNUsZ1e52GGPUYqEk5N6N7GhTCWO3YH9NVa4oKPa6yBw2jTQ/QVYQmc0kd7HyrjYxVaOo6xf9IggFHQkJ1BMOWVzq7rFK5rECsoYL793zAVIqXAtmud";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        VuforiaLocalizer vuforia;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        OpenGLMatrix dab = OpenGLMatrix
                .translation(6,0,0);
        waitForStart();

        while (opModeIsActive()) {
            VectorF translation = dab.getTranslation();
            double TranslationX = translation.get(0);
            telemetry.addData("66", TranslationX);


        }


    }

}


