package org.firstinspires.ftc.teamcode.subsystems;

public class intake {
    hMap hwMap = new hMap();
    public void teleopIntake(double intakePower) {
        hwMap.intake.setPower(intakePower);
    }
}