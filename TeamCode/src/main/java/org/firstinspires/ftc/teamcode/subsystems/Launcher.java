package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Launcher {
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public DcMotorEx pollenLauncher = null;
    public DcMotorEx nectarLauncher = null;

    public double POLLEN_RPM = 3700;
    public double NECTAR_RPM = 3700;
    public static final double TICKS_PER_REVOLUTION = 28;

    public static final double RPM_ACCEPTED_RANGE = 100;
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

    public boolean atVelocity() {
        double actualPollenRPM = pollenLauncher.getVelocity() * 60 / TICKS_PER_REVOLUTION;
        double actualNectarRPM = nectarLauncher.getVelocity() * 60 / TICKS_PER_REVOLUTION;

        return launcherMode == LauncherMode.ON
                        && POLLEN_RPM > 0
                        && NECTAR_RPM > 0
                        && Math.abs(actualPollenRPM - POLLEN_RPM) <= RPM_ACCEPTED_RANGE
                        && Math.abs(actualNectarRPM - NECTAR_RPM) <= RPM_ACCEPTED_RANGE;
    }
}
