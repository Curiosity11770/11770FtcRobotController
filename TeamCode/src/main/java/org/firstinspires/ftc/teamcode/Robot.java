package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Scoring;

public class Robot {
    private LinearOpMode myOpMode = null;

    public Drivetrain drivetrain;
    public Intake intake;
    public Extension extension;
    public Lift lift;
    public Scoring scoring;

    public Robot(LinearOpMode opmode){
        myOpMode = opmode;
    }
    public void init() {
        drivetrain = new Drivetrain(myOpMode);
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
        drivetrain.teleOp();
        lift.teleOp();
        extension.teleOp();
        intake.teleOp();
        scoring.teleOp();


    }

    public void driveToHuskyLens(){
        double xTarget = 200;
        double rightMostX = 0;
        int rightMostIndex = 0;
        PIDController strafeController;
        strafeController = new PIDController(Drivetrain.DRIVE_KP, Drivetrain.DRIVE_KI, Drivetrain.DRIVE_KD, Drivetrain.DRIVE_MAX_OUT);
        HuskyLens.Block[] blocks = drivetrain.huskylens.blocks();
        myOpMode.telemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++) {
            myOpMode.telemetry.addData("Block", blocks[i].toString());
            if(blocks[i].x > rightMostX){
                rightMostX = blocks[i].x;
                rightMostIndex = i;
                extension.leftLinkPosition = (424.53-blocks[i].y)/380.74;
            }

        }
        myOpMode.telemetry.addData("RightMostX", rightMostX);
        if(rightMostX < xTarget){
            double strafePower = strafeController.calculate(xTarget, rightMostX);
            drivetrain.leftFrontDrive.setPower(-strafePower);
            drivetrain.leftBackDrive.setPower(strafePower);
            drivetrain.rightFrontDrive.setPower(strafePower);
            drivetrain.rightBackDrive.setPower(-strafePower);
            myOpMode.telemetry.addData("StrafePower", strafePower);
        }else{
            drivetrain.leftFrontDrive.setPower(0);
            drivetrain.leftBackDrive.setPower(0);
            drivetrain.rightFrontDrive.setPower(0);
            drivetrain.rightBackDrive.setPower(0);
        }
    }

    public void update(){
        drivetrain.update();
    }

}
