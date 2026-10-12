package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    // servos: axon
    // motors: gobilda (1)

    private OpMode myOpMode = null;

    public Servo pivot = null;
    public DcMotor intake = null;

    public enum IntakeMode {
        IN,
        OUT,
        OFF
    }

    public IntakeMode intakeMode = IntakeMode.OFF;
    public double INTAKE_SPEED = -1;

    public Intake(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        intake = myOpMode.hardwareMap.get(DcMotor.class, "intake");
        pivot = myOpMode.hardwareMap.get(Servo.class, "pivot");

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
    }

}
