package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * @version 1.0
 * @author Ishan_Arora
 * Main drive train class for an tank drive
 */
public class TankDriveTrain {

    //syncronized singleton
    private static TankDriveTrain driveTrainTank = null;
    public static TankDriveTrain getNewDriveInstance(HardwareMap hwMap) {
        if(driveTrainTank == null) {
            driveTrainTank = new TankDriveTrain(hwMap);
        }
        return driveTrainTank;
    }

    //outputs
    private double[] motorOutputs;
//    private boolean isDone;
  //  private double forwardSetpoint;
    //private double strafeSetpoint;


    private DcMotor Left;
    private DcMotor Right;

    //possible states the drive train could be in
    private systemStates possState;
    private enum systemStates  {
        SYSTEM_OFF , DRIVE, AUTO
    };

    public TankDriveTrain(HardwareMap hwMap) {

        //sets up the enum state case
        possState = systemStates.SYSTEM_OFF;

        //drive motor outputs
        // isDone = false;
        motorOutputs = new double[2];
      ///   forwardSetpoint = 0;
         //strafeSetpoint = 0;


        //front drive motors
        Left = hwMap.get(DcMotor.class, "left");
        //back drive motors
        Right = hwMap.get(DcMotor.class, "right");
       int position = Left.getCurrentPosition();
 int positionright = Right.getCurrentPosition();




    }

    public void updateTank() {
        switch (possState) {
            case SYSTEM_OFF:
                Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                updateDriveTank(0,0);
                break;
            case DRIVE:
                break;
            case AUTO:
          Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            default:
                System.out.println("Yo, you messed up enumeration within drivetrain dude");
        }

        if(possState != systemStates.AUTO) {
        //sets the power for the  motors
        Left.setPower(motorOutputs[0]);
        Right.setPower(-motorOutputs[1]);

        }
    }

    /**
     * Updates the x drive base
     * @param drive  : robots forwards and backwards movement
     * @param turn   : rotational value
     */


    public void updateDriveTank(double drive , double turn) {

        //enables motor control
        possState = systemStates.DRIVE;

        motorOutputs[0] = drive + turn;
        motorOutputs[1] = drive - turn;
    }

//       public boolean isDone() {
//            return isDone;
//        }
    public void setStop()
        {
        possState = systemStates.SYSTEM_OFF;
        this.updateTank();
    }
    public void forwardTimes(int times){
        Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.updateDriveTank(1,0);
        this.updateTank();
        int pos = times * 1440;
        Left.setTargetPosition(pos);
        Right.setTargetPosition(pos);
        Left.getCurrentPosition();
        while(Left.isBusy()){

        }
        this.setStop();


    }



    /**
     *
     * @param forwards : CM
     */
   // public void setAutoForwards(double forwards) {
     //   this.forwardSetpoint = forwards;
//    }
//
//    /**
//     *
//     * @param strafe : CM
//     */
//    public void setAutoStrafe(double strafe) {
//        this.forwardSetpoint = 0;
//        this.strafeSetpoint = strafe;
//    }


}
