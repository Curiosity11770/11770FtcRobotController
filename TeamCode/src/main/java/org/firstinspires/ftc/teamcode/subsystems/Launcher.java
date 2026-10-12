package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    // motors: pollenLauncher, nectarLauncher
    // servos: pollenGate, nectarGate

    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public DcMotorEx pollenLauncher = null;
    public DcMotorEx nectarLauncher = null;
    public AnalogInput turretEncoder = null;

    public double FAR = 1;
    public double CLOSE = 1;
    public double FARRPM = 1;
    public double CLOSERPM = 1;
    public double measuredRPM;

    public double revolutions_per_minute = 5000;
    public static final double TICKS_PER_REVOLUTION = 28;
    double TICKS_PER_SECOND = revolutions_per_minute / 60 * TICKS_PER_REVOLUTION;

    public enum LauncherMode {
        ON,
        OFF,
        AUTO,
    }

    public LauncherMode launcherMode = LauncherMode.OFF;

    public Launcher (OpMode opmode) {
        myOpMode = opmode;
    }
    public void init(){
        pollenLauncher = myOpMode.hardwareMap.get(DcMotorEx.class, "pollenLauncher");
        pollenLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenLauncher.setDirection(DcMotor.Direction.REVERSE);

        nectarLauncher = myOpMode.hardwareMap.get(DcMotorEx.class, "nectarLauncher");
        nectarLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        nectarLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarLauncher.setDirection(DcMotor.Direction.REVERSE);

        turretEncoder = myOpMode.hardwareMap.get(AnalogInput.class, "turretEncoder");

        myOpMode.telemetry.addData(">", "Launcher Initialized");


    }

    public void update(){
        TICKS_PER_SECOND = (revolutions_per_minute / 60) * TICKS_PER_REVOLUTION;

        if (launcherMode == LauncherMode.ON) {
            pollenLauncher.setVelocity(TICKS_PER_SECOND);
            nectarLauncher.setVelocity(TICKS_PER_SECOND);
        } else if(launcherMode == LauncherMode.OFF) {
            pollenLauncher.setVelocity(0);
            nectarLauncher.setVelocity(0);
        }
    }
}
