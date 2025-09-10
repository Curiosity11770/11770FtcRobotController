package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.motionProfile.ProfileDrivetrain;
import org.firstinspires.ftc.teamcode.utility.PIDController;


public class Robot {
    private LinearOpMode myOpMode = null;

    public ProfileDrivetrain drivetrain;
    public Intake intake;
    public Extension extension;
    public Lift lift;
    public Scoring scoring;

    public boolean isDriving = true;
    public ElapsedTime timer = new ElapsedTime();
    double leftMostY = 0;

    double constant = 0.001;

    public Robot(LinearOpMode opmode){
        myOpMode = opmode;
    }
    public void init() {
        drivetrain = new ProfileDrivetrain(myOpMode);
        intake = new Intake(myOpMode);
        lift = new Lift(myOpMode);
        extension = new Extension (myOpMode);
        scoring = new Scoring (myOpMode);

        drivetrain.init();
        intake.init();
        lift.init();
        extension.init();
        scoring.init();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
    public void teleOp() {
        drivetrain.update();
        lift.teleOp();
        extension.teleOp();
        intake.teleOp();
        scoring.teleOp();
    }

    public void driveToHuskyLens(){
            double xTarget = 70;
            double leftMostX = 320;
            int leftMostIndex = 0;
            PIDController strafeController;
            strafeController = new PIDController(0.025, Drivetrain.DRIVE_KI, Drivetrain.DRIVE_KD, 0.3);
            HuskyLens.Block[] blocks = drivetrain.huskylens.blocks();
            myOpMode.telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                myOpMode.telemetry.addData("Block", blocks[i].toString());

                if (blocks[i].x < leftMostX) {
                    leftMostX = blocks[i].x;
                    leftMostIndex = i;
                    leftMostY = blocks[leftMostIndex].y;
                    extension.update();
                }

            }
            myOpMode.telemetry.addData("LeftMostY", leftMostY);
            myOpMode.telemetry.addData("LeftMostX", leftMostX);
            if (leftMostX > xTarget && isDriving) {
                double strafePower = strafeController.calculate(xTarget, leftMostX);
                drivetrain.leftFrontDrive.setPower(-strafePower);
                drivetrain.leftBackDrive.setPower(strafePower);
                drivetrain.rightFrontDrive.setPower(strafePower);
                drivetrain.rightBackDrive.setPower(-strafePower);
                myOpMode.telemetry.addData("StrafePower", strafePower);
            } else {
                drivetrain.leftFrontDrive.setPower(0);
                drivetrain.leftBackDrive.setPower(0);
                drivetrain.rightFrontDrive.setPower(0);
                drivetrain.rightBackDrive.setPower(0);
                extension.leftLinkPosition = ((98.21 + leftMostY) / 427.01) + 0.05;
                extension.rightLinkPosition = ((98.21 + leftMostY) / 427.01) + 0.05;
                isDriving = false;
            }
            timer.reset();
    }

    public void submersibleIntake(){
            intake.spinIntake.setPower(1);
            drivetrain.driveTime(0.3, 2.0, timer);
            intake.update();
            intake.sensorUpdate();
            extension.update();
            constant += 0.004;
            extension.leftLinkPosition = ((98.21 + leftMostY) / 427.01) - constant - 0.05;
            extension.rightLinkPosition = ((98.21 + leftMostY) / 427.01) - constant - 0.05;
            if (intake.colors.green >= 0.01){
            }
    }

    public void update(){
        drivetrain.update();
    }

}
