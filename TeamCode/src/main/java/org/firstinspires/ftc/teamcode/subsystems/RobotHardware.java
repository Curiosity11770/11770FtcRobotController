package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;


public class RobotHardware {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    ElapsedTime runtime = new ElapsedTime();

    //TODO Add instance of Vision subsystem...either Webcam, HuskyLens, or Limelight
    public Drivetrain drivetrain;
    public Intake intake;
    public Launcher launcher;
    public Transfer transfer;


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        //drivetrain = new Drivetrain(myOpMode);
        intake = new Intake(myOpMode);
        launcher = new Launcher(myOpMode);
        transfer = new Transfer(myOpMode);

        //drivetrain.init();
        intake.init();
        launcher.init();
        transfer.init();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
    }

    public void teleOp() {
        intake.update();
        launcher.update();
        transfer.update();

        // Intake/Transfer Teleop
        if (myOpMode.gamepad1.right_bumper) {
            intake.intakePivotMode = Intake.IntakePivotMode.DOWN;
            intake.intakeMode = Intake.IntakeMode.IN;
            transfer.transferMode = Transfer.TransferMode.ON;
        } else if (myOpMode.gamepad1.left_bumper) {
            intake.intakeMode = Intake.IntakeMode.OUT;
        } else {
            intake.intakePivotMode = Intake.IntakePivotMode.UP;
            intake.intakeMode = Intake.IntakeMode.OFF;
            transfer.transferMode = Transfer.TransferMode.OFF;
        }

        if (myOpMode.gamepad1.right_trigger > 0.5) {
            transfer.transferMode = Transfer.TransferMode.ON;
            launcher.launcherMode = Launcher.LauncherMode.ON;
            if (launcher.atVelocity()) {
                transfer.gateMode = Transfer.GateMode.OPEN;
            } else {
                transfer.gateMode = Transfer.GateMode.CLOSE;
            }
        } else {
            transfer.gateMode = Transfer.GateMode.CLOSE;
        }

        // Launcher Teleop
        if (myOpMode.gamepad1.y) {
            launcher.launcherMode = Launcher.LauncherMode.ON;
        } else if (myOpMode.gamepad1.a) {
            launcher.launcherMode = Launcher.LauncherMode.OFF;
        }

        if (myOpMode.gamepad1.dpad_up) {
            intake.flowerMode = Intake.FlowerMode.UP;
        } else if (myOpMode.gamepad1.dpad_down) {
            intake.flowerMode = Intake.FlowerMode.DOWN;
        }

        // RPM Test
        if (myOpMode.gamepad2.dpad_up) {
            launcher.POLLEN_RPM += 100;
        } else if (myOpMode.gamepad2.dpad_down) {
            launcher.POLLEN_RPM -= 100;
        }

        if (myOpMode.gamepad2.dpad_right) {
            launcher.NECTAR_RPM += 100;
        } else if (myOpMode.gamepad2.dpad_left) {
            launcher.NECTAR_RPM -= 100;
        }
    }

    public void update() {
        intake.update();
        launcher.update();
        transfer.update();
    }
}