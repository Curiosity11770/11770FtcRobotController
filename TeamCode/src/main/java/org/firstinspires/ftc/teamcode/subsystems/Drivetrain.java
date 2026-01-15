package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.OpModes.FieldOrientedTeleop;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utility.PIDController;

import java.util.List;

public class Drivetrain {
    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;

    private LinearOpMode myOpMode = null;

    private Follower follower;

    private double slowModeMultiplier = 0.5;

    private Vision vision;

    public PIDController turnPID = null;


    enum DriveMode{
        ROBOT_CENTRIC,
        RED_FIELD_CENTRIC,
        BLUE_FIELD_CENTRIC
    }

    public boolean autoTurn = false;
    public double turnInput = 0;
    DriveMode driveMode = DriveMode.ROBOT_CENTRIC;

    public double turnKP = 0.018;
    public double turnKI = 0;
    public double turnKD = 0;

    public Drivetrain(LinearOpMode opmode, Vision robotVision) {

        myOpMode = opmode;
        vision = robotVision;
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
        turnPID = new PIDController(turnKP, turnKI, turnKD, 0.8);


    }

    public void teleOp() {

        follower.update();

        if (myOpMode.gamepad1.left_trigger > 0.2) {
            vision.limelight.pipelineSwitch(5);
            autoTurn = true;
        } else if (myOpMode.gamepad1.left_trigger > 0.2) {
            vision.limelight.pipelineSwitch(0);
            autoTurn = true;
        } else {
            autoTurn = false;
        }
        if (!autoTurn) {
            turnInput = -myOpMode.gamepad1.right_stick_x * slowModeMultiplier;
        } else if (autoTurn) {

            vision.result = vision.limelight.getLatestResult();
            if (vision.result.isValid()) {

                // Access general information
                Pose3D botpose = vision.result.getBotpose();
                double captureLatency = vision.result.getCaptureLatency();
                double targetingLatency = vision.result.getTargetingLatency();
                double parseLatency = vision.result.getParseLatency();

                myOpMode.telemetry.addData("tx", vision.result.getTx());
                myOpMode.telemetry.addData("txnc", vision.result.getTxNC());
                myOpMode.telemetry.addData("ty", vision.result.getTy());
                myOpMode.telemetry.addData("tync", vision.result.getTyNC());


                myOpMode.telemetry.addData("Botpose", botpose.toString());

                // Access barcode results
                List<LLResultTypes.BarcodeResult> barcodeResults = vision.result.getBarcodeResults();
                for (LLResultTypes.BarcodeResult br : barcodeResults) {
                    myOpMode.telemetry.addData("Barcode", "Data: %s", br.getData());

                }

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = vision.result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    myOpMode.telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    myOpMode.telemetry.addData("targetPose", fr.getTargetPoseRobotSpace());
                    myOpMode.telemetry.addData("cameraPose", fr.getTargetPoseCameraSpace());

                }

                double turnPower = turnPID.calculate(0, vision.result.getTx());
                myOpMode.telemetry.addData("turnPower", turnPower);

                    turnInput = turnPower;


            }
        }
            if (myOpMode.gamepad1.x) {
                driveMode = DriveMode.ROBOT_CENTRIC;
            } else if (myOpMode.gamepad1.b) {
                vision.limelight.pipelineSwitch(5);
                driveMode = DriveMode.BLUE_FIELD_CENTRIC;
            } else if (myOpMode.gamepad1.y) {
                vision.limelight.pipelineSwitch(0);
                driveMode = DriveMode.RED_FIELD_CENTRIC;

            }

        List<LLResultTypes.FiducialResult> fiducialResults = vision.result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            myOpMode.telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            myOpMode.telemetry.addData("targetPose", fr.getTargetPoseRobotSpace());
            myOpMode.telemetry.addData("cameraPose", fr.getTargetPoseCameraSpace());

        }

            if (driveMode == DriveMode.ROBOT_CENTRIC) {
                follower.setTeleOpDrive(
                        -myOpMode.gamepad1.left_stick_y * slowModeMultiplier,
                        -myOpMode.gamepad1.left_stick_x * slowModeMultiplier,
                        turnInput
                );
            } else if (driveMode == DriveMode.RED_FIELD_CENTRIC) {
                follower.setTeleOpDrive(
                        myOpMode.gamepad1.left_stick_y * slowModeMultiplier,
                        myOpMode.gamepad1.left_stick_x * slowModeMultiplier,
                        turnInput,
                        false
                );
            } else if (driveMode == DriveMode.BLUE_FIELD_CENTRIC) {
                follower.setTeleOpDrive(
                        -myOpMode.gamepad1.left_stick_y * slowModeMultiplier,
                        -myOpMode.gamepad1.left_stick_x * slowModeMultiplier,
                        turnInput,
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
