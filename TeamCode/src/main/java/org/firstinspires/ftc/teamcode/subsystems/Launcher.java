package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import com.pedropathing.ivy.Command;
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

    public Command launcherOn(double rpm) {
        double ticksPerSecond = (rpm / 60) * TICKS_PER_REVOLUTION;
        ElapsedTime timer = new ElapsedTime();

        return Command.build()
                .setStart(() -> {
                    nectarLauncher.setVelocity(ticksPerSecond);
                    pollenLauncher.setVelocity(ticksPerSecond);
                    timer.reset();
                })
                .setExecute(() -> {
                    myOpMode.telemetry.addData("pollen measured RPM", pollenLauncher.getVelocity() * 60 / TICKS_PER_REVOLUTION);
                    myOpMode.telemetry.addData("nectar measured RPM", nectarLauncher.getVelocity() * 60 / TICKS_PER_REVOLUTION);
                })
                // old: keep running while (velocity < target && time < 2.5)
                .setDone(() -> pollenLauncher.getVelocity() >= ticksPerSecond || timer.seconds() >= 2.5);
    }

    public Command launcherOff() {
        return Command.build()
                .setStart(() -> {
                    nectarLauncher.setVelocity(0);
                    pollenLauncher.setVelocity(0);
                })
                // old: keep running while (nectar > 0 || pollen > 0)
                .setDone(() -> nectarLauncher.getVelocity() <= 0 && pollenLauncher.getVelocity() <= 0);
    }
}
