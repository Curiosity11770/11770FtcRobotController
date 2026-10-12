package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {

    // motors: TransferMotor
    // servos: NectarGate, PollenGate

    private OpMode myOpMode = null;

    public Servo nectarGate = null;
    public Servo pollenGate = null;
    public DcMotor transferMotor = null;

    public enum TransferMode {
        ON,
        OFF,
    }

    public Transfer.TransferMode transferMode = Transfer.TransferMode.OFF;

    public Transfer(OpMode opmode) {
        myOpMode = opmode;

    }

    public void init(){

        nectarGate = myOpMode.hardwareMap.get(Servo.class, "nectarGate");
        pollenGate = myOpMode.hardwareMap.get(Servo.class, "pollenGate");
        transferMotor = myOpMode.hardwareMap.get(DcMotor.class, "transferMotor");

        myOpMode.telemetry.addData(">", "Transfer Initialized");

    }
    public void update(){}

}
