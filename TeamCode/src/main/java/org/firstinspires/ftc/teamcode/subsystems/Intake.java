package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    private LinearOpMode myOpMode = null;

    public DcMotor intakeMotor = null;



    public Intake(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        intakeMotor = myOpMode.hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setPower(0);

    }

    public void teleOp(){
        if(myOpMode.gamepad2.left_trigger > 0.2){
            intakeMotor.setPower(1);
        }else if(myOpMode.gamepad2.right_trigger > 0.2){
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }
    }

    public void update(){
    }


}
