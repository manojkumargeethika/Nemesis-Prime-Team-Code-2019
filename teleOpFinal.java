package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.hMap;

@TeleOp(name="roverRuckusTeleop", group="Linear Opmode")
public class  teleOpFinal extends LinearOpMode {
    public org.firstinspires.ftc.teamcode.subsystems.driveTrain dTrain = new org.firstinspires.ftc.teamcode.subsystems.driveTrain();
    public org.firstinspires.ftc.teamcode.subsystems.intake intake = new org.firstinspires.ftc.teamcode.subsystems.intake();
    public hMap hwMap= new hMap();

    public void runOpMode() {
        hwMap.init(hardwareMap);


        waitForStart();
        double intakePower;
        double speedlimiter;
        if (gamepad1.right_bumper) {
            speedlimiter = 1.0;
        } else {
            speedlimiter = 0.75;
        }
        if (gamepad1.right_trigger >= 0.2 || gamepad2.a) {
            intakePower = 1.0;
        } else if (gamepad1.left_trigger >= 0.2 || gamepad2.b) {
            intakePower = -1.0;
        } else {
            intakePower = 0.0;
        }
        dTrain.teleopInput(gamepad1.left_stick_y,gamepad1.right_stick_x,speedlimiter);
        intake.teleopIntake(intakePower);

    }
}
