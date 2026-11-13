package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Launcher {
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public DcMotorEx pollenLauncher = null;
    public DcMotorEx nectarLauncher = null;

    public double POLLEN_RPM = 3620;
    public double NECTAR_RPM = 3700;
    public static final double TICKS_PER_REVOLUTION = 28;

    public static final double RPM_ACCEPTED_RANGE = 25;
    double POLLEN_TICKS_PER_SECOND = POLLEN_RPM / 60 * TICKS_PER_REVOLUTION;
    double NECTAR_TICKS_PER_SECOND = NECTAR_RPM / 60 * TICKS_PER_REVOLUTION;

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
        pollenLauncher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pollenLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pollenLauncher.setDirection(DcMotorEx.Direction.REVERSE);

        nectarLauncher = myOpMode.hardwareMap.get(DcMotorEx.class, "nectarLauncher");
        nectarLauncher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        nectarLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        nectarLauncher.setDirection(DcMotorEx.Direction.REVERSE);

        myOpMode.telemetry.addData(">", "Launcher Initialized");
    }

    public void update(){
        POLLEN_TICKS_PER_SECOND = (POLLEN_RPM / 60) * TICKS_PER_REVOLUTION;
        NECTAR_TICKS_PER_SECOND = (NECTAR_RPM / 60) * TICKS_PER_REVOLUTION;

        if (launcherMode == LauncherMode.ON) {
            pollenLauncher.setVelocity(POLLEN_TICKS_PER_SECOND);
            nectarLauncher.setVelocity(NECTAR_TICKS_PER_SECOND);
        } else if (launcherMode == LauncherMode.OFF) {
            pollenLauncher.setVelocity(0);
            nectarLauncher.setVelocity(0);
        }
    }

    public boolean atVelocity(DcMotorEx motor, double RPM) {
        double actualRPM = motor.getVelocity() * 60 / TICKS_PER_REVOLUTION;

        return launcherMode == LauncherMode.ON && Math.abs(actualRPM - RPM) <= RPM_ACCEPTED_RANGE;
    }

    public Action launcherOn(double rpm) {
        return new Action() {
            private boolean initialized = false;
            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    TICKS_PER_SECOND = (rpm / 60) * TICKS_PER_REVOLUTION;
                    spin1.setVelocity(TICKS_PER_SECOND);
                    spin2.setVelocity(TICKS_PER_SECOND);
                    initialized = true;
                    timer.reset();
                }

                double measuredRPM = spin1.getVelocity() * 60 / TICKS_PER_REVOLUTION;
                myOpMode.telemetry.addData("measuredRPM2", measuredRPM);
                myOpMode.telemetry.addData("ticks per second", spin1.getVelocity());

                myOpMode.telemetry.update();

                return spin1.getVelocity() < TICKS_PER_SECOND && timer.seconds() < 2.5;
            }
        };
    }

    public Action launcherOff() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    spin1.setVelocity(0);
                    spin2.setVelocity(0);
                    initialized = true;
                }
                return spin1.getVelocity() > 0;
            }
        };
    }
}
