package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    // servos: axon
    // motors: gobilda (1)

    private OpMode myOpMode = null;

    public Servo pivot = null;
    public Servo flower = null;
    public DcMotor intake = null;

    public enum IntakeMode {
        IN,
        OUT,
        OFF
    }

    public enum IntakePivotMode {
        UP,
        DOWN
    }

    public enum FlowerMode {
        UP,
        DOWN
    }

    public static final double PIVOT_UP = 1;
    public static final double PIVOT_DOWN = 0.5;
    public static final double FLOWER_UP = 0.95;
    public static final double FLOWER_DOWN = 0.18;
    public static final double INTAKE_SPEED = 1;

    public IntakeMode intakeMode = IntakeMode.OFF;
    public IntakePivotMode intakePivotMode = IntakePivotMode.UP;
    public FlowerMode flowerMode = FlowerMode.UP;

    public Intake(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        intake = myOpMode.hardwareMap.get(DcMotor.class, "intake");
        pivot = myOpMode.hardwareMap.get(Servo.class, "intakePivot");
        flower = myOpMode.hardwareMap.get(Servo.class, "flowerServo");

        myOpMode.telemetry.addData(">", "Intake Initialized");
    }

    public void update() {
        if (intakeMode == IntakeMode.IN) {
            intake.setPower(INTAKE_SPEED);
        } else if (intakeMode == IntakeMode.OUT) {
            intake.setPower(-INTAKE_SPEED);
        } else {
            intake.setPower(0);
        }

        if (intakePivotMode == IntakePivotMode.UP) {
            pivot.setPosition(PIVOT_UP);
        } else if (intakePivotMode == IntakePivotMode.DOWN) {
            pivot.setPosition(PIVOT_DOWN);
        }

        if (flowerMode == FlowerMode.UP) {
            flower.setPosition(FLOWER_UP);
        } else if (flowerMode == FlowerMode.DOWN) {
            flower.setPosition(FLOWER_DOWN);
        }
    }

    public void teleOp() {
        update();
    }

}


