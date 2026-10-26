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
//        drivetrain = new Drivetrain(myOpMode);
        intake = new Intake(myOpMode);
        launcher = new Launcher(myOpMode);
        transfer = new Transfer(myOpMode);

//        drivetrain.init();
        intake.init();
        launcher.init();
        transfer.init();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
    }

    public void teleOp() {
//        drivetrain.teleOp();
        intake.update();
        launcher.update();
        transfer.update();

        //intake teleop
        // b = REVERSE
        // x = ON
        if (myOpMode.gamepad1.b) {
            intake.intakeMode = Intake.IntakeMode.OUT;
        } else if (myOpMode.gamepad1.x) {
            intake.intakeMode = Intake.IntakeMode.IN;
        } else {
            intake.intakeMode = Intake.IntakeMode.OFF;
        }

        // launcher teleop
        // y = ON
        // a = OFF
        if (myOpMode.gamepad1.y) {
            launcher.launcherMode = Launcher.LauncherMode.ON;
        } else if (myOpMode.gamepad1.a) {
            launcher.launcherMode = Launcher.LauncherMode.OFF;
        }

    }

    public void update() {
        intake.update();
        launcher.update();
        transfer.update();
    }
}
    /*public void code(){
        print("Hello world!");

        Artifacts = true;
        Auto = works;
        Fail = no;
        Worlds = true;
    }
    */
