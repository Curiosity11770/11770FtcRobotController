package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class Intake {
    private LinearOpMode myOpMode = null;
    public DcMotor spinIntake = null;
    public Servo flipIntake = null;

    public static final double SPIN_SPEED = 0.7;
    public static final double INTAKE_CLOSED = 0.3;
    public static final double INTAKE_OPEN = 0.49;

    public double flipPosition;

    public Intake(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){

        spinIntake = myOpMode.hardwareMap.get(DcMotor.class, "spinIntake");
        flipIntake = myOpMode.hardwareMap.get(Servo.class, "flipIntake");

        spinIntake.setPower(0);
        myOpMode.telemetry.addData("intake", flipIntake.getPosition());
        //flipIntake.setPosition(INTAKE_CLOSED);
        flipPosition = INTAKE_OPEN;
    }

    public void teleOp(){
        flipIntake.setPosition(flipPosition);
        if(myOpMode.gamepad2.a) {
            flipPosition = INTAKE_OPEN;
        } else if (myOpMode.gamepad2.b){
            flipPosition = INTAKE_CLOSED;
        }
        if(myOpMode.gamepad2.left_trigger > 0.7) {
            spinIntake.setPower(1);
        } else if (myOpMode.gamepad2.right_trigger > 0.7){
            spinIntake.setPower(-1);
        } else {
            spinIntake.setPower(0);
        }
    }

}
