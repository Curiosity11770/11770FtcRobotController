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
    public double transferRPM = 500;
    public enum GateMode {
        OPEN,
        CLOSE,
    }

    public enum TransferMode {
        ON,
        OFF,
    }

    public Transfer.GateMode gateMode = Transfer.GateMode.CLOSE;
    public Transfer.TransferMode transferMode = Transfer.TransferMode.ON;

    public Transfer(OpMode opmode) {
        myOpMode = opmode;

    }

    public void init(){

        nectarGate = myOpMode.hardwareMap.get(Servo.class, "nectarGate");
        pollenGate = myOpMode.hardwareMap.get(Servo.class, "pollenGate");
        transferMotor = myOpMode.hardwareMap.get(DcMotor.class, "transferMotor");

        myOpMode.telemetry.addData(">", "Transfer Initialized");

    }
    public void update(){
        if(gateMode == GateMode.CLOSE){
            nectarGate.setPosition(0);
            pollenGate.setPosition(0);
        } else if (gateMode == GateMode.OPEN){
            nectarGate.setPosition(1);
            pollenGate.setPosition(1);
        }

        if(transferMode == TransferMode.OFF){
            transferMotor.setPower(transferRPM);
        } else transferMotor.setPower(0);
    }

}
