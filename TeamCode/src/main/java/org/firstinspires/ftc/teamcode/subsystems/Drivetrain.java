package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Drivetrain {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;
    GoBildaPinpointDriver pinpoint;
    public IMU imu;

    Pose2D pose2D;

    static final double COUNTS_PER_INCH = 50;
    ElapsedTime timeout = new ElapsedTime();

    public String col = null;

    public Drivetrain(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        pinpoint = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, Math.toRadians(0)));

        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        //pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        myOpMode.telemetry.addData(">", "Drivetrain Initialized");
    }

    public void teleOp() {
        pinpoint.update();
        pose2D = pinpoint.getPosition();

        myOpMode.telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        myOpMode.telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        myOpMode.telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));

        double max;

        double drive = -myOpMode.gamepad1.left_stick_y;
        double turn = myOpMode.gamepad1.right_stick_x;
        double strafe = -myOpMode.gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 2);

        double frontLeftPower = (drive + turn - strafe) / denominator;
        double frontRightPower = (drive - turn + strafe) / denominator;
        double backLeftPower = (drive + turn + strafe) / denominator;
        double backRightPower = (drive - turn - strafe) / denominator;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        leftFrontDrive.setPower(frontLeftPower * 1.3);
        rightFrontDrive.setPower(frontRightPower * 1.3);
        leftBackDrive.setPower(backLeftPower * 1.3);
        rightBackDrive.setPower(backRightPower * 1.3);
    }

    public void resetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void useEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void turnCCW (double motorPower, double degrees){
        // optionally reset imu heading - don't do this if you need field coordinates
        imu.resetYaw();
        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        //calculate the target angle based on current angle
        double targetAngle = orientation.getYaw(AngleUnit.DEGREES)+degrees;

        //start the motors
        leftFrontDrive.setPower(-motorPower);
        leftBackDrive.setPower(-motorPower);
        rightFrontDrive.setPower(motorPower);
        rightBackDrive.setPower(motorPower);

        //update orientation and check angle
        while (myOpMode.opModeIsActive() && targetAngle > orientation.getYaw(AngleUnit.DEGREES)){
            // Update Rotational Angles
            orientation = imu.getRobotYawPitchRollAngles();

            myOpMode.telemetry.addData("IMU position: ", orientation.getYaw(AngleUnit.DEGREES));
            myOpMode.telemetry.addData("Target Angle: ", targetAngle);
            myOpMode.telemetry.update();
        }

        //stop the motors
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void turnCW (double motorPower, double degrees){
        // optionally reset imu heading - don't do this if you need field coordinates
        imu.resetYaw();
        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        //calculate the target angle based on current angle
        double targetAngle = orientation.getYaw(AngleUnit.DEGREES)-degrees;

        //start the motors
        leftFrontDrive.setPower(motorPower);
        leftBackDrive.setPower(motorPower);
        rightFrontDrive.setPower(-motorPower);
        rightBackDrive.setPower(-motorPower);

        //update orientation and check angle
        while (myOpMode.opModeIsActive() && targetAngle < orientation.getYaw(AngleUnit.DEGREES)){
            // Update Rotational Angles
            orientation = imu.getRobotYawPitchRollAngles();

            myOpMode.telemetry.addData("IMU position: ", orientation.getYaw(AngleUnit.DEGREES));
            myOpMode.telemetry.addData("Target Angle: ", targetAngle);
        }

        //stop the motors
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void encoderDrive(double speed, double distance, double timeoutS) {
        int targetCounts;

        // Ensure that the opmode is still active
        if (myOpMode.opModeIsActive()) {

            //Reset the motor encoders so they start from 0
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            targetCounts = (int)(distance * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(targetCounts);
            leftBackDrive.setTargetPosition(targetCounts);
            rightFrontDrive.setTargetPosition(targetCounts);
            rightBackDrive.setTargetPosition(targetCounts);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            timeout.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (myOpMode.opModeIsActive() &&
                    (timeout.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy())) {

                // Display it for the driver.
                myOpMode.telemetry.addData("Running to", targetCounts);
                myOpMode.telemetry.addData("LeftFront at", leftFrontDrive.getCurrentPosition());
                myOpMode.telemetry.addData("LeftBack at", leftBackDrive.getCurrentPosition());
                myOpMode.telemetry.addData("RightFront at", rightFrontDrive.getCurrentPosition());
                myOpMode.telemetry.addData("RightBack at", rightBackDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void autoStrafe (double speed, double distance, double timeoutS){
        int targetCounts;

        // Ensure that the opmode is still active
        if (myOpMode.opModeIsActive()) {

            //Reset the motor encoders so they start from 0
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Determine new target position, and pass to motor controller
        targetCounts = (int)(distance * COUNTS_PER_INCH);
        leftFrontDrive.setTargetPosition(targetCounts);
        leftBackDrive.setTargetPosition(-targetCounts);
        rightFrontDrive.setTargetPosition(-targetCounts);
        rightBackDrive.setTargetPosition(targetCounts);

        // Turn On RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        timeout.reset();
        leftFrontDrive.setPower(Math.abs(speed));
        leftBackDrive.setPower(Math.abs(speed));
        rightFrontDrive.setPower(Math.abs(speed));
        rightBackDrive.setPower(Math.abs(speed));

        while (myOpMode.opModeIsActive() && (timeout.seconds() < timeoutS) && (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {
            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", targetCounts);
            myOpMode.telemetry.addData("LeftFront at", leftFrontDrive.getCurrentPosition());
            myOpMode.telemetry.addData("LeftBack at", leftBackDrive.getCurrentPosition());
            myOpMode.telemetry.addData("RightFront at", rightFrontDrive.getCurrentPosition());
            myOpMode.telemetry.addData("RightBack at", rightBackDrive.getCurrentPosition());
            myOpMode.telemetry.update();
        }

        // Stop all motion;
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        // Turn On RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(-72.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }
}