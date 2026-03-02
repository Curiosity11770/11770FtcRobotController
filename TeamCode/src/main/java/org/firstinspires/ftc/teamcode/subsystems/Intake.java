package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
            intakeMotor.setPower(0.5);
        }else if(myOpMode.gamepad2.right_trigger > 0.2){
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }
    }

    public void update(){
    }

    public Action intakeOn(double time) {
        ElapsedTime actionTimer = new ElapsedTime();
        actionTimer.reset();
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    actionTimer.reset();
                    intakeMotor.setPower(-1);
                    initialized = true;
                }
                return actionTimer.seconds() < time;
            }
        };
    }


}
