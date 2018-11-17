package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.Range;

public class driveTrain {
    hMap hwMap = new hMap();


   public void teleopInput(double drive, double turn, double speedLimiter) {
       double leftPower;
       double rightPower;
       leftPower = Range.clip(drive + turn, -speedLimiter, speedLimiter);
       rightPower = Range.clip(drive - turn, -speedLimiter, speedLimiter);
       hwMap.leftDrive.setPower(leftPower);
       hwMap.rightDrive.setPower(rightPower);
   }
}
