package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utility.PIDController;

import java.util.List;



public class Drivetrain {
    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;

    PIDController headingController;

    private LinearOpMode myOpMode = null;

    private Follower follower;

    private double slowModeMultiplier = 0.5;

    private Vision vision;

    public PIDController turnPID = null;

    public GoBildaPinpointDriver pinpoint;


    enum DriveMode{
        ROBOT_CENTRIC,
        RED_FIELD_CENTRIC,
        BLUE_FIELD_CENTRIC
    }

     enum SideMode {
        RED,
        BLUE
    }

    public boolean autoTurn = false;
    public double turnInput = 0;
    DriveMode driveMode = DriveMode.ROBOT_CENTRIC;

    public double turnKP = 0.025;
    public double turnKI = 0;
    public double turnKD = 0;

    private boolean isHoldingPosition = false;
    private int currentPipeline = 1;

    public double roboLocationX;
    public double roboLocationY;

    public SideMode side = SideMode.BLUE;

    public static double HEADING_KP = 0.005;
    public static double HEADING_KI = 0;
    public static double HEADING_KD = 0;
    public static double MAX_OUT = 0.8;


    public Drivetrain(LinearOpMode opmode, Vision robotVision) {

        myOpMode = opmode;
        vision = robotVision;
    }

    public Drivetrain(LinearOpMode opmode) {

        myOpMode = opmode;
    }

    public void init(){

        headingController = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD, MAX_OUT);

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

        pinpoint = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.setOffsets(-5, 8, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.recalibrateIMU();

        if(myOpMode.gamepad1.left_bumper){
            side = SideMode.RED;
            myOpMode.telemetry.addData(">", "RED");
        }else if(myOpMode.gamepad1.right_bumper){
            side = SideMode.BLUE;
            myOpMode.telemetry.addData(">", "BLUE");
        }


        follower = Constants.createFollower(myOpMode.hardwareMap);
        follower.update();
        myOpMode.telemetry.addData("Status", "Waiting for Start");
        myOpMode.telemetry.update();
        follower.startTeleopDrive(true);
        turnPID = new PIDController(turnKP, turnKI, turnKD, 0.8);


    }

    public void teleOp() {

        follower.update();

        if(myOpMode.gamepad1.left_bumper){
            side = SideMode.RED;
            myOpMode.telemetry.addData(">", "RED");
        }else if(myOpMode.gamepad1.right_bumper){
            side = SideMode.BLUE;
            myOpMode.telemetry.addData(">", "BLUE");
        }

        roboLocationX = pinpoint.getPosX(DistanceUnit.INCH);
        roboLocationY = pinpoint.getPosY(DistanceUnit.INCH);

        if (myOpMode.gamepad1.right_trigger > 0.2) {
            if (currentPipeline != 5) {
                vision.limelight.pipelineSwitch(5);
                currentPipeline = 5;
            }
            autoTurn = true;
        } else if (myOpMode.gamepad1.left_trigger > 0.2) {
            if (currentPipeline != 0) {
                vision.limelight.pipelineSwitch(0);
                currentPipeline = 0;
            }
            autoTurn = true;
        } else {
            autoTurn = false;
        }
        if (!autoTurn) {
            turnInput = -myOpMode.gamepad1.right_stick_x * slowModeMultiplier;
        } else {

            vision.result = vision.limelight.getLatestResult();

                // Access general information
               /* Pose3D botpose = vision.result.getBotpose();
                double captureLatency = vision.result.getCaptureLatency();
                double targetingLatency = vision.result.getTargetingLatency();
                double parseLatency = vision.result.getParseLatency();*/

               /* myOpMode.telemetry.addData("tx", vision.result.getTx());
                myOpMode.telemetry.addData("txnc", vision.result.getTxNC());
                myOpMode.telemetry.addData("ty", vision.result.getTy());
                myOpMode.telemetry.addData("tync", vision.result.getTyNC());


                myOpMode.telemetry.addData("Botpose", botpose.toString());*/

                /*// Access barcode results
                List<LLResultTypes.BarcodeResult> barcodeResults = vision.result.getBarcodeResults();
                for (LLResultTypes.BarcodeResult br : barcodeResults) {
                    myOpMode.telemetry.addData("Barcode", "Data: %s", br.getData());

                }*/

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = vision.result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    //myOpMode.telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    //myOpMode.telemetry.addData("targetPose", fr.getTargetPoseRobotSpace());
                    myOpMode.telemetry.addData("cameraPose", fr.getTargetPoseCameraSpace());

                }
                myOpMode.telemetry.addData("Result", vision.result.isValid());
                myOpMode.telemetry.addData("Tx", vision.result.getTx());
                if (vision.result.isValid()) {
                    double turnPower = turnPID.calculate(0, vision.result.getTx());
                    turnInput = turnPower;
                    //myOpMode.telemetry.addData("turnPower", turnPower);
                } else {
                    turnInput = 0;
                }




        }
        /*if(myOpMode.gamepad1.dpad_right) {

            if (side == SideMode.RED) {
                double adjustedError = angleWrap(142 - pinpoint.getHeading(AngleUnit.DEGREES));
                turnInput = -headingController.calculate(adjustedError);

            }
        }

        if(myOpMode.gamepad1.dpad_left) {
                double adjustedError = angleWrap(38 - pinpoint.getHeading(AngleUnit.DEGREES));
                turnInput = -headingController.calculate(adjustedError);
            } else if (side == SideMode.BLUE) {
            follower.holdPoint(follower.getPose());
            isHoldingPosition = true;
            myOpMode.telemetry.addData("Status", "HOLDING POSITION (Active Braking)");
            return;
        }*/

        if (isHoldingPosition) {
            follower.startTeleopDrive(true);
            isHoldingPosition = false;
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

        /*List<LLResultTypes.FiducialResult> fiducialResults = vision.result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            myOpMode.telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            myOpMode.telemetry.addData("targetPose", fr.getTargetPoseRobotSpace());
            myOpMode.telemetry.addData("cameraPose", fr.getTargetPoseCameraSpace());

        }*/

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
                slowModeMultiplier = 1;
            } else {
                slowModeMultiplier = 1;
            }

            myOpMode.telemetry.addData("Drive Mode: ", driveMode);

    }

    public void teleOpExtra() {
        //Drivetrain TeleOp Code
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -myOpMode.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  myOpMode.gamepad1.left_stick_x;
        double yaw     =  myOpMode.gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void update(){

    }

    public double angleWrap(double degrees) {

        while (degrees > 180) {
            degrees -= 360;
        }
        while (degrees < -180) {
            degrees += 360;
        }

        // keep in mind that the result is in degrees
        return degrees;
    }

}
