package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Action;

public class Shooter {
    private LinearOpMode myOpMode = null;
    private DcMotor shootingMotor = null;
    private CRServo linkageShooting = null;
    private Servo transferServo = null;

    public Shooter (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init (){
        shootingMotor = myOpMode.hardwareMap.get(DcMotor.class, "shootingMotor");
        transferServo = myOpMode.hardwareMap.get(Servo.class, "transferServo");
        linkageShooting = myOpMode.hardwareMap.get(CRServo.class, "linkageServo");

    }
    public Action shooting(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }
        };
    }

    public void teleOp(){
        if(myOpMode.gamepad2.right_bumper){
        }else if(myOpMode.gamepad2.left_bumper){
        } else {
        }
    }
}
