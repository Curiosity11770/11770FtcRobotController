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
    public static final double TRANSFER_POWER = 1;

    public enum GateMode {
        OPEN,
        CLOSE,
    }

    public enum TransferMode {
        ON,
        OFF,
    }

    public static final double POLLEN_GATE_OPEN = 0.61;
    public static final double POLLEN_GATE_CLOSED = 0.5;
    public static final double NECTAR_GATE_OPEN = 0.72;
    public static final double NECTAR_GATE_CLOSED = 0.5;

    public Transfer.GateMode gateMode = Transfer.GateMode.CLOSE;
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
    public void update(){
        if (gateMode == GateMode.CLOSE){
            nectarGate.setPosition(NECTAR_GATE_CLOSED);
            pollenGate.setPosition(POLLEN_GATE_CLOSED);
        } else if (gateMode == GateMode.OPEN){
            nectarGate.setPosition(NECTAR_GATE_OPEN);
            pollenGate.setPosition(POLLEN_GATE_OPEN);
        }

        if (transferMode == TransferMode.ON) {
            transferMotor.setPower(TRANSFER_POWER);
        } else if (transferMode == TransferMode.OFF) {
            transferMotor.setPower(0);
        }
    }

}
