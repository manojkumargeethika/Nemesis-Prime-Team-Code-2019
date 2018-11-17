package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.hMap;


@Autonomous(name="Auto_Final", group="Blank Auto" )
public class auton_final extends LinearOpMode {

    static double i_position = 1.0; //initial position for drop servo
    static double f_position = 0.6; //final position
    public Servo lockServo;

    @Override
    public void runOpMode() throws InterruptedException {

        //for elevator
        hMap hwMap = new hMap();
        controlServo();



    }
    public void controlServo(){
        lockServo.setPosition(0.5);



    }


}
