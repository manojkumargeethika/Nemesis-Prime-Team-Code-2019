package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class hMap {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor intake = null;
    HardwareMap hwMap           =  null;

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        intake  = hwMap.get(DcMotor.class, "intake");
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        intake.setPower(0);
    }
}
