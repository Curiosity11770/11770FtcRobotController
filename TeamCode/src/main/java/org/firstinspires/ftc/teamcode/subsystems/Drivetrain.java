package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OpModes.FieldOrientedTeleop;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drivetrain {
    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;

    private LinearOpMode myOpMode = null;

    private Follower follower;

    private double slowModeMultiplier = 0.5;


    enum DriveMode{
        ROBOT_CENTRIC,
        RED_FIELD_CENTRIC,
        BLUE_FIELD_CENTRIC
    }
    DriveMode driveMode = DriveMode.ROBOT_CENTRIC;

    public Drivetrain(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        follower = Constants.createFollower(myOpMode.hardwareMap);
        follower.update();
        myOpMode.telemetry.addData("Status", "Waiting for Start");
        myOpMode.telemetry.update();
        follower.startTeleopDrive(true);

    }

    public void teleOp(){

        follower.update();

        if (myOpMode.gamepad1.x) {
            driveMode = DriveMode.ROBOT_CENTRIC;
        } else if (myOpMode.gamepad1.b) {
            driveMode = DriveMode.BLUE_FIELD_CENTRIC;
        } else if (myOpMode.gamepad1.y) {
            driveMode = DriveMode.RED_FIELD_CENTRIC;

        }

        if (driveMode == DriveMode.ROBOT_CENTRIC) {
            follower.setTeleOpDrive(
                    -myOpMode.gamepad1.left_stick_y*slowModeMultiplier,
                    -myOpMode.gamepad1.left_stick_x*slowModeMultiplier,
                    -myOpMode.gamepad1.right_stick_x*slowModeMultiplier
            );
        } else if (driveMode == DriveMode.RED_FIELD_CENTRIC) {
            follower.setTeleOpDrive(
                    -myOpMode.gamepad1.left_stick_y*slowModeMultiplier,
                    -myOpMode.gamepad1.left_stick_x*slowModeMultiplier,
                    -myOpMode.gamepad1.right_stick_x*slowModeMultiplier,
                    false
            );
        } else if (driveMode == DriveMode.BLUE_FIELD_CENTRIC) {
            follower.setTeleOpDrive(
                    myOpMode.gamepad1.left_stick_y*slowModeMultiplier,
                    -myOpMode.gamepad1.left_stick_x*slowModeMultiplier,
                    -myOpMode.gamepad1.right_stick_x*slowModeMultiplier,
                    false
            );
        }

        if (myOpMode.gamepad1.left_bumper) {
            slowModeMultiplier = 0.25;
        } else if (myOpMode.gamepad1.right_bumper) {
            slowModeMultiplier = 0.9;
        } else {
            slowModeMultiplier = 0.5;
        }

        myOpMode.telemetry.addData("Drive Mode: ", driveMode);

    }

    public void update(){

    }

}
