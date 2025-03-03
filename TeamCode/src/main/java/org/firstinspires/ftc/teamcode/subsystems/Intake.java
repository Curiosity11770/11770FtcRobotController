package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class Intake {
    private LinearOpMode myOpMode = null;
    public DcMotor spinIntake = null;
    public Servo flipIntake = null;

    public static final double INTAKE_UP = 0.34;
    public static final double INTAKE_DOWN = 0.6;
    public static final double INTAKE_STOWED = 0;

    public boolean transfer = false;

    public double flipPosition;

    public enum IntakeMode {
        TRANSFER,
        STOWED,
        INTAKE
    }
    public IntakeMode intakeMode = IntakeMode.STOWED;

    public Intake(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){

        spinIntake = myOpMode.hardwareMap.get(DcMotor.class, "spinIntake");
        flipIntake = myOpMode.hardwareMap.get(Servo.class, "flipIntake");

        spinIntake.setPower(0);
        myOpMode.telemetry.addData("intake", flipIntake.getPosition());
        flipPosition = INTAKE_STOWED;
        flipIntake.setPosition(INTAKE_STOWED);
    }

    public void teleOp(){
        update();

        if(myOpMode.gamepad2.b){
            intakeMode = IntakeMode.TRANSFER;
        }else if(myOpMode.gamepad2.a){
            intakeMode = IntakeMode.INTAKE;
        } else if (myOpMode.gamepad2.dpad_left){
            intakeMode = IntakeMode.TRANSFER;
            spinIntake.setPower(1);
        }

        /*
        if(myOpMode.gamepad2.a) {
            intakeMode = IntakeMode.INTAKE;
        } else if (myOpMode.gamepad2.b){
            intakeMode = IntakeMode.TRANSFER;
        }
        */


        if(myOpMode.gamepad2.left_trigger > 0.7) {
            spinIntake.setPower(1);
            transfer = false;
        } else if (myOpMode.gamepad2.right_trigger > 0.7){
            spinIntake.setPower(-1);
            transfer = false;
        } else if (myOpMode.gamepad2.dpad_left) {
            spinIntake.setPower(1);
            transfer = true;
        /*} else if (transfer == true){
            spinIntake.setPower(1);*/
        } else {
            spinIntake.setPower(0);
        }
    }

    public void update(){
        flipIntake.setPosition(flipPosition);
        if (intakeMode == IntakeMode.INTAKE){
            flipPosition = INTAKE_DOWN;
        } else if (intakeMode == IntakeMode.TRANSFER){
            spinIntake.setPower(0);
            flipPosition = INTAKE_UP;
        } else if(intakeMode == IntakeMode.STOWED){
            flipPosition = INTAKE_UP;
        }
    }

}
