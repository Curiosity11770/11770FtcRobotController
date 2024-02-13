package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

public class Robot {

    public boolean passed, notPassed;
    private LinearOpMode myOpMode = null;

    public Drivetrain drivetrain;

    public Lift lift;

    public Intake intake;

    public Scoring scoring;

    public Drone drone;

    public DualPortalCameras camera;

    //Variables for AprilTag Motion --> eventually reconcile these with drive constants in Drivetrain class
    final double SPEED_GAIN  =  0.03  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.02 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.02  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    public Robot(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public void init(){
        drivetrain = new Drivetrain(myOpMode);
        lift = new Lift(myOpMode);
        intake = new Intake(myOpMode);
        scoring = new Scoring(myOpMode);
        drone = new Drone(myOpMode);
        camera = new DualPortalCameras(myOpMode);

        myOpMode.telemetry.addData("IsWorking", drone);

        drivetrain.init();
        lift.init();
        intake.init();
        scoring.init();
        drone.init();
        camera.init();
    }

    public void teleOp(){
        drivetrain.teleOp();
        lift.teleOp();
        intake.teleOp();
        scoring.teleOp(passed, notPassed);
        drone.teleOp();

        if(drivetrain.state == Drivetrain.DriveMode.APRILTAGS){
            driveToAprilTagTeleOp(drivetrain.AprilTagTarget, 8);
        }

        if(intake.frontPixel && intake.backPixel){
           scoring.rightGateServo.setPosition(scoring.GATE_DOWN_RIGHT);
            scoring.leftGateServo.setPosition(scoring.GATE_DOWN_LEFT);
        }

        if(myOpMode.gamepad2.dpad_right){
            lift.liftMode = Lift.LiftMode.LOW;
            scoring.state = Scoring.ScoringMode.SCORING;
            scoring.timer.reset();
            if(scoring.timer.seconds() > 1 && scoring.state == Scoring.ScoringMode.SCORING){
                scoring.boxServo.setPosition(scoring.BOX_OUT);
            }

        }

        if(myOpMode.gamepad2.dpad_down){
            lift.liftMode = Lift.LiftMode.INTAKE;
        }
    }

    void driveToAprilTag(int targetTag, double targetDistance) {
        double rangeError = 100;
        double drive = 0;
        double turn = 0;
        double strafe = 0;

        while (rangeError > 1 && myOpMode.opModeIsActive()) {
            camera.scanAprilTag(targetTag);
            if (camera.targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                rangeError = (camera.desiredTag.ftcPose.range - targetDistance);
                double headingError = camera.desiredTag.ftcPose.bearing;
                double yawError = camera.desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED)*-1;
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                // Calculate wheel powers.
                double leftFrontPower = drive - strafe - turn;
                double rightFrontPower = drive + strafe + turn;
                double leftBackPower = drive + strafe - turn;
                double rightBackPower = drive - strafe + turn;

                // Normalize wheel powers to be less than 1.0
                double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                // Send powers to the wheels.
                drivetrain.driveFrontLeft.setPower(leftFrontPower);
                drivetrain.driveFrontRight.setPower(rightFrontPower);
                drivetrain.driveBackLeft.setPower(leftBackPower);
                drivetrain.driveBackRight.setPower(rightBackPower);
            }else{
                drivetrain.stopMotors();
            }
            drivetrain.localizer.update();
            drivetrain.localizer.updateDashboard();
            myOpMode.telemetry.update();
        }

        drivetrain.stopMotors();
    }

    void driveToAprilTagTeleOp(int targetTag, double targetDistance) {
        double rangeError = 0;
        double drive = 0;
        double turn = 0;
        double strafe = 0;

            camera.scanAprilTag(targetTag);
            if (camera.targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                rangeError = (camera.desiredTag.ftcPose.range - targetDistance);
                double headingError = camera.desiredTag.ftcPose.bearing;
                double yawError = camera.desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED)*-1;
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                // Calculate wheel powers.
                double leftFrontPower = drive - strafe - turn;
                double rightFrontPower = drive + strafe + turn;
                double leftBackPower = drive + strafe - turn;
                double rightBackPower = drive - strafe + turn;

                // Normalize wheel powers to be less than 1.0
                double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                // Send powers to the wheels.
                drivetrain.driveFrontLeft.setPower(leftFrontPower);
                drivetrain.driveFrontRight.setPower(rightFrontPower);
                drivetrain.driveBackLeft.setPower(leftBackPower);
                drivetrain.driveBackRight.setPower(rightBackPower);
            }else{
                drivetrain.stopMotors();
            }
            drivetrain.localizer.update();
            drivetrain.localizer.updateDashboard();
            myOpMode.telemetry.update();

       // drivetrain.stopMotors();
    }

    /*public void touchSense(){
        if(!lift.getTouch()){
            intake.intakeLeft.setPower(0);
            intake.intakeRight.setPower(0);
            intake.intakeMotor.setPower(0);
        }
    }*/
}
