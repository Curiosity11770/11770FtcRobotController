package org.firstinspires.ftc.teamcode.motionProfile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utility.LowPassFilter;
import org.firstinspires.ftc.teamcode.utility.PIDController;
import org.firstinspires.ftc.teamcode.utility.PinPointLocalizer;

import java.util.Locale;

@Config
public class ProfileDrivetrain {
    private LinearOpMode myOpMode = null;   // Reference to the calling OpMode.
    public PinPointLocalizer localizer;
    ElapsedTime time = new ElapsedTime();

    public HuskyLens huskylens;

    public Servo sweeper;

    // Drivetrain motors
    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;

    //standard PID controllers for "AUTO" mode
    PIDController xController;
    PIDController yController;
    PIDController headingController;

    // Outer loop (position) PID controllers for Profile following.
    PIDController profileXController;
    PIDController profileYController;

    // Inner loop (velocity) PID controllers.
    PIDController xVelController;
    PIDController yVelController;

    LowPassFilter velocityFilterX = new LowPassFilter(0.3);
    LowPassFilter velocityFilterY = new LowPassFilter(0.3);

    public MotionProfile motionProfile;

    Pose2D targetPose;
    public boolean targetReached = false;

    // standard gains for "AUTO" mode – tune these for your system.
    public static double HEADING_KP = 0.015;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.0;
    public static double DRIVE_KP = 0.03;
    public static double DRIVE_KI = 0.0;
    public static double DRIVE_KD = 0.0;
    public static double DRIVE_MAX_OUT = 0.7;

    // Gains for the inner velocity loop.
    public static double PROFILE_KP = 9;
    public static double PROFILE_KI = 0.0;
    public static double PROFILE_KD = 0.0;

    // Gains for the inner velocity loop.
    public static double VELOCITY_KP = 0.05;
    public static double VELOCITY_KI = 0.0;
    public static double VELOCITY_KD = 0.0;

    // Feedforward gain for acceleration.
    public static double FEEDFORWARD_ACC = 0.003;
    public static double FEEDFORWARD_VEL = 0.0175;

    // Maximum velocity and acceleration for the motion profile.
    public static double DRIVE_MAX_ACC = 50;
    public static double DRIVE_MAX_VEL = 50;

    // Other constants.
    public static double STRAFE_MULTIPLIER = 2;

    //used to adjust smoothing factor of low pass filter
    public static double FILTER_CONSTANT = 1;

    public static double MAX_ACCEL_FILTER = 10;
    public static double LOW_PASS_ALPHA = 1;

    double prevMotorCmdX = 0;
    double prevMotorCmdY = 0;

    double prevMotorCmdx2 = 0;
    double prevMotorCmdy2 = 0;

    public enum DrivetrainMode {
        MANUAL,
        AUTO,
        PROFILE,
        BACKUP,
        IDLE,
    }
    public DrivetrainMode drivetrainMode = DrivetrainMode.MANUAL;

    public ProfileDrivetrain(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        // Initialize PID controllers.
        xController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_MAX_OUT);
        yController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_MAX_OUT);
        headingController = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD, DRIVE_MAX_OUT);

        profileXController = new PIDController(PROFILE_KP,PROFILE_KI,PROFILE_KD, DRIVE_MAX_VEL);
        profileYController = new PIDController(PROFILE_KP,PROFILE_KI,PROFILE_KD, DRIVE_MAX_VEL);

        xVelController = new PIDController(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD, DRIVE_MAX_VEL);
        yVelController = new PIDController(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD, DRIVE_MAX_VEL);

        // Initialize motion profile with dummy values; it will be set when a target is given.
        motionProfile = new MotionProfile(0, 0, 0, 0, DRIVE_MAX_VEL, DRIVE_MAX_ACC);
        targetPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        targetReached = true;

        localizer = new PinPointLocalizer(myOpMode);
        localizer.init();
        localizer.update();

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

        resetEncoders();
        runWithoutEncoders();

        sweeper = myOpMode.hardwareMap.get(Servo.class, "sweeper");
        sweeper.setPosition(0.7);
        huskylens = myOpMode.hardwareMap.get(HuskyLens.class, "huskyLens");
        huskylens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        drivetrainMode = DrivetrainMode.MANUAL;

        myOpMode.telemetry.addData(">", "Drivetrain Initialized");
    }

    public void resetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runWithoutEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetPose(Pose2D newTarget){
        targetPose = newTarget;
        targetReached = false;
        drivetrainMode = DrivetrainMode.AUTO;
    }

    public void profiledDriveToTarget(double xTarget, double yTarget, double degreeTarget) {
        // Check if the target pose is new.
        if (targetPose.getX(DistanceUnit.INCH) != xTarget ||
                targetPose.getY(DistanceUnit.INCH) != yTarget ||
                targetPose.getHeading(AngleUnit.DEGREES) != degreeTarget) {

            drivetrainMode = DrivetrainMode.PROFILE;
            targetPose = new Pose2D(DistanceUnit.INCH, xTarget, yTarget, AngleUnit.DEGREES, degreeTarget);
            targetReached = false;
            // Create a new motion profile from current position to target.
            motionProfile = new MotionProfile(localizer.getX(), localizer.getY(), xTarget, yTarget, DRIVE_MAX_VEL, DRIVE_MAX_ACC);
        }
    }

    public void profiledRelativeDriveToTarget(double relativeXTarget, double relativeYTarget, double relativeDegreeTarget){
        drivetrainMode = DrivetrainMode.PROFILE;

        double cosTheta = Math.cos(Math.toRadians(localizer.getHeading()));
        double sinTheta = Math.sin(Math.toRadians(localizer.getHeading()));

        double newX = localizer.getX() + relativeXTarget * cosTheta - relativeYTarget * sinTheta;
        double newY = localizer.getY() + relativeXTarget * sinTheta + relativeYTarget * cosTheta;
        double newTheta = localizer.getHeading() + relativeDegreeTarget;

        targetPose = new Pose2D(DistanceUnit.INCH, newX, newY, AngleUnit.DEGREES, newTheta);
        targetReached = false;
        // Create a new motion profile from current position to target.
        motionProfile = new MotionProfile(localizer.getX(), localizer.getY(), newX, newY, DRIVE_MAX_VEL, DRIVE_MAX_ACC);
    }

    public void driveTime(double speed, double seconds, ElapsedTime timer) {
        drivetrainMode = DrivetrainMode.BACKUP;
        if(timer.seconds() < seconds) {
            leftBackDrive.setPower(speed);
            leftFrontDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            rightBackDrive.setPower(speed);
        } else {
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
    }

    public void driveStraightTime(double motorPower, double time, ElapsedTime timer){
        localizer.update();
        if (time < timer.seconds()){
            leftFrontDrive.setPower(motorPower);
            rightFrontDrive.setPower(motorPower);
            rightBackDrive.setPower(motorPower);
            leftBackDrive.setPower(motorPower);
        } else {
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
        }
    }


    public void relativeDriveToTarget(double relativeXTarget, double relativeYTarget, double relativeDegreeTarget, double kpValue){
        drivetrainMode = DrivetrainMode.AUTO;
        xController = new PIDController(kpValue, DRIVE_KI, DRIVE_KD, DRIVE_MAX_OUT);
        yController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_MAX_OUT);

        double cosTheta = Math.cos(Math.toRadians(localizer.getHeading()));
        double sinTheta = Math.sin(Math.toRadians(localizer.getHeading()));

        double newX = localizer.getX() + relativeXTarget * cosTheta - relativeYTarget * sinTheta;
        double newY = localizer.getY() + relativeXTarget * sinTheta + relativeYTarget * cosTheta;
        double newTheta = localizer.getHeading() + relativeDegreeTarget;

        targetPose = new Pose2D(DistanceUnit.INCH, newX, newY, AngleUnit.DEGREES, newTheta);
        targetReached = false;
        // Create a new motion profile from current position to target.
        //motionProfile = new MotionProfile(localizer.getX(), localizer.getY(), newX, newY, DRIVE_MAX_VEL, DRIVE_MAX_ACC);
    }

    public double angleWrap(double degrees) {
        while (degrees > 180) {
            degrees -= 360;
        }
        while (degrees < -180) {
            degrees += 360;
        }
        return degrees;
    }

    public void update(){
        localizer.update();

        // Update PID gains if needed.
        xController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_MAX_OUT);
        yController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_MAX_OUT);
        headingController = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD, DRIVE_MAX_OUT);

        profileXController = new PIDController(PROFILE_KP,PROFILE_KI,PROFILE_KD, DRIVE_MAX_VEL);
        profileYController = new PIDController(PROFILE_KP,PROFILE_KI,PROFILE_KD, DRIVE_MAX_VEL);

        xVelController = new PIDController(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD, DRIVE_MAX_VEL);
        yVelController = new PIDController(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD, DRIVE_MAX_VEL);

        double feedforward_accel = FEEDFORWARD_ACC;
        double feedforward_vel = FEEDFORWARD_VEL;

        if(drivetrainMode == DrivetrainMode.MANUAL){
            //sweeper
            if(myOpMode.gamepad1.left_bumper){
                sweeper.setPosition(0.1);
            } else {
                sweeper.setPosition(0.7);
            }

            //drive train
            double max;

            double leftFrontPower;
            double rightFrontPower;
            double leftBackPower;
            double rightBackPower;

            double drive = -myOpMode.gamepad1.left_stick_y;
            double turn = myOpMode.gamepad1.right_stick_x;
            double strafe = -myOpMode.gamepad1.left_stick_x;

            leftFrontPower = (drive + turn - strafe);
            rightFrontPower = (drive - turn + strafe);
            leftBackPower = (drive + turn + strafe);
            rightBackPower = (drive - turn - strafe);

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            //Slow and Turbo Buttons
            if (myOpMode.gamepad1.left_trigger > 0.3) {
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            } else if (myOpMode.gamepad1.right_trigger > 0.3) {
                leftFrontDrive.setPower(leftFrontPower / 7);
                rightFrontDrive.setPower(rightFrontPower / 7);
                leftBackDrive.setPower(leftBackPower / 7);
                rightBackDrive.setPower(rightBackPower / 7);
            } else {
                leftFrontDrive.setPower(leftFrontPower / 2);
                rightFrontDrive.setPower(rightFrontPower / 2);
                leftBackDrive.setPower(leftBackPower / 2);
                rightBackDrive.setPower(rightBackPower / 2);
            }
        } else if (drivetrainMode == DrivetrainMode.PROFILE) {
            // Retrieve target state from the motion profile.
            // targetState: {targetX, targetY, targetVelocity, targetAcceleration}
            double[] targetState = motionProfile.getTargetState();
            //inches
            double instantX = targetState[0];
            double instantY = targetState[1];
            //in/s
            double targetVel = targetState[2];
            //in/(s*s)
            double targetAcc = targetState[3];

            // Compute the unit vector along the profile path.
            double pathDX = motionProfile.targetX - motionProfile.startX;
            double pathDY = motionProfile.targetY - motionProfile.startY;
            double pathDist = Math.hypot(pathDX, pathDY);
            double ux = (pathDist == 0) ? 0 : pathDX / pathDist;
            double uy = (pathDist == 0) ? 0 : pathDY / pathDist;

            // Desired velocity and acceleration components.
            double desiredVx = targetVel * ux;
            double desiredVy = targetVel * uy;
            double desiredAx = targetAcc * ux;
            double desiredAy = targetAcc * uy;

            // --- Outer Loop: Position Correction ---
            double posErrorX = instantX - localizer.getX();
            double posErrorY = instantY - localizer.getY();
            double velAdjX = profileXController.calculate(instantX, localizer.getX());
            double velAdjY = profileXController.calculate(instantY, localizer.getY());

            // Combine feedforward desired velocity with position corrections.
            double targetVx = desiredVx + velAdjX;
            double targetVy = desiredVy + velAdjY;

            // --- Inner Loop: Velocity Control ---
            // Obtain the measured velocity from your odometry.
            //Pose2D currentVel = localizer.odo.getVelocity();
            //double currentVx = currentVel.getX(DistanceUnit.INCH);
            //double currentVy = currentVel.getY(DistanceUnit.INCH);

            // Inside your update loop
            //Obtains raw velocity from the pinpoint computer
            double rawVelX = localizer.odo.getVelocity().getX(DistanceUnit.INCH);
            double rawVelY = localizer.odo.getVelocity().getY(DistanceUnit.INCH);

            //filters the raw data through the low pass filter
            velocityFilterX.alpha = FILTER_CONSTANT;
            velocityFilterY.alpha = FILTER_CONSTANT;

            double filteredVelX = velocityFilterX.filter(rawVelX);
            double filteredVelY = velocityFilterY.filter(rawVelY);

            //double velErrorX = targetVx - filteredVelX;
            //double velErrorY = targetVy - filteredVelY;

            // Calculate inner loop outputs plus feedforward velocity.
            //motor power should be between -1 and 1
            double motorCmdX = xVelController.calculate(targetVx, filteredVelX) + feedforward_vel * desiredVx + feedforward_accel * desiredAx;
            double motorCmdY = yVelController.calculate(targetVy, filteredVelY) + feedforward_vel * desiredVy + feedforward_accel * desiredAy;

            //secondary low pass filter applied to the motor command
            double alpha = LOW_PASS_ALPHA;  // Smoothing factor (adjust between 0.5 - 0.9) lower value = more smoothing
            motorCmdX = alpha * motorCmdX + (1 - alpha) * prevMotorCmdX;
            prevMotorCmdX = motorCmdX;

            motorCmdY = alpha * motorCmdY + (1 - alpha) * prevMotorCmdY;
            prevMotorCmdY = motorCmdY;

            //Limits the acceleration of the motor itself, so it can’t change its value that much
            double maxAcceleration = MAX_ACCEL_FILTER; // Tune this based on robot behavior
            double deltaCmdX = motorCmdX - prevMotorCmdx2;
            deltaCmdX = Range.clip(deltaCmdX, -maxAcceleration, maxAcceleration);
            motorCmdX = prevMotorCmdx2 + deltaCmdX;
            prevMotorCmdx2 = motorCmdX;

            double deltaCmdY = motorCmdY - prevMotorCmdy2;
            deltaCmdY = Range.clip(deltaCmdY, -maxAcceleration, maxAcceleration);
            motorCmdY = prevMotorCmdy2 + deltaCmdY;
            prevMotorCmdy2 = motorCmdY;


            // --- Heading Control ---
            double wrappedAngleError = angleWrap(targetPose.getHeading(AngleUnit.DEGREES) - localizer.getHeading());
            double tPower = headingController.calculate(wrappedAngleError);

            // Rotate the computed field-centric motor commands based on the robot's heading.
            double radianHeading = Math.toRadians(localizer.getHeading());
            double xPower_rotated = motorCmdX * Math.cos(-radianHeading) - motorCmdY * Math.sin(-radianHeading);
            double yPower_rotated = motorCmdX * Math.sin(-radianHeading) + motorCmdY * Math.cos(-radianHeading);

            // Mix the components into wheel commands.
            double leftFrontPower = xPower_rotated - yPower_rotated  - tPower;
            double leftBackPower = xPower_rotated + yPower_rotated  - tPower;
            double rightFrontPower = xPower_rotated + yPower_rotated  + tPower;
            double rightBackPower = xPower_rotated - yPower_rotated + tPower;

            // Find the maximum absolute power value
            double maxPower = Math.max(
                    Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                    Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))
            );

// Scale if the max power exceeds 1
            if (maxPower > 1.0) {
                leftFrontPower /= maxPower;
                leftBackPower /= maxPower;
                rightFrontPower /= maxPower;
                rightBackPower /= maxPower;
            }

            // Apply power limits.
            leftFrontDrive.setPower(Range.clip(leftFrontPower, -1, 1));
            leftBackDrive.setPower(Range.clip(leftBackPower, -1, 1));
            rightFrontDrive.setPower(Range.clip(rightFrontPower, -1, 1));
            rightBackDrive.setPower(Range.clip(rightBackPower, -1, 1));

            // Determine if the profile is complete.
            targetReached = motionProfile.profileComplete();
                    //&& headingController.targetReached;

            // Telemetry for debugging.
            myOpMode.telemetry.addData("Profile Phase", motionProfile.phase);
            myOpMode.telemetry.addData("DRIVE_KP", DRIVE_KP);
            myOpMode.telemetry.addData("VELOCITY_KP", VELOCITY_KP);
            myOpMode.telemetry.addData("Feedforward Acc", feedforward_accel);
            myOpMode.telemetry.addData("Feedforward Vel", feedforward_accel);
            myOpMode.telemetry.addData("Filter Constant", FILTER_CONSTANT);
            myOpMode.telemetry.addData("Low Pass alpha", LOW_PASS_ALPHA);
            myOpMode.telemetry.addData("Target Vx", targetVx);
            myOpMode.telemetry.addData("Target Vy", targetVy);
            myOpMode.telemetry.addData("MotorCmdX", motorCmdX);
            myOpMode.telemetry.addData("MotorCmdY", motorCmdY);

            myOpMode.telemetry.addData("Profile Complete", motionProfile.profileComplete());

            FtcDashboard dashboard = FtcDashboard.getInstance();
            Telemetry dashboardTelemetry = dashboard.getTelemetry();

            dashboardTelemetry.addData("instantX (from profile)", instantX);
            dashboardTelemetry.addData("rawX (from profile)", rawVelX);
            dashboardTelemetry.addData("currentX (from pinpoint)", localizer.getX());
            dashboardTelemetry.addData("desiredXVel (from profile)", desiredVx);
            dashboardTelemetry.addData("velocityAdjX (positional error output)", velAdjX);
            dashboardTelemetry.addData("targetVX (profile V + positional error correction)", targetVx);
            dashboardTelemetry.addData("filteredXVelocity", filteredVelX);
            dashboardTelemetry.addData("feedforward*desiredAX", (feedforward_accel * desiredAx));
            dashboardTelemetry.addData("feedforward*desiredVX", (feedforward_vel * desiredVx));
            dashboardTelemetry.addData("motorCMDX (inner loop velocity error + feedforward)", motorCmdX);
            dashboardTelemetry.update();

        }else if(drivetrainMode == DrivetrainMode.AUTO) {
            //Use PIDs to calculate motor powers based on error to targets
            double xPower = xController.calculate(targetPose.getX(DistanceUnit.INCH), localizer.getX());
            double yPower = yController.calculate(targetPose.getY(DistanceUnit.INCH), localizer.getY());

            double wrappedAngleError = angleWrap(targetPose.getHeading(AngleUnit.DEGREES) - localizer.getHeading());
            double tPower = headingController.calculate(wrappedAngleError);

            double radianHeading = Math.toRadians(localizer.getHeading());

            //rotate the motor powers based on robot heading
            double xPower_rotated = xPower * Math.cos(-radianHeading) - yPower * Math.sin(-radianHeading);
            double yPower_rotated = xPower * Math.sin(-radianHeading) + yPower * Math.cos(-radianHeading);

            // Mix the components into wheel commands.
            double leftFrontPower = xPower_rotated - yPower_rotated * STRAFE_MULTIPLIER  - tPower;
            double leftBackPower = xPower_rotated + yPower_rotated * STRAFE_MULTIPLIER - tPower;
            double rightFrontPower = xPower_rotated + yPower_rotated * STRAFE_MULTIPLIER + tPower;
            double rightBackPower = xPower_rotated - yPower_rotated * STRAFE_MULTIPLIER + tPower;

            // Find the maximum absolute power value
            double maxPower = Math.max(
                    Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                    Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))
            );

// Scale if the max power exceeds 1
            if (maxPower > 1.0) {
                leftFrontPower /= maxPower;
                leftBackPower /= maxPower;
                rightFrontPower /= maxPower;
                rightBackPower /= maxPower;
            }

            // x, y, theta input mixing to deliver motor powers
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);

            //TODO Create a function that checks if target is reached based on target pose and current position
            //check if drivetrain is still working towards target
            targetReached = (xController.targetReached && yController.targetReached && headingController.targetReached);
            String data = String.format(Locale.US, "{tX: %.3f, tY: %.3f, tH: %.3f}", targetPose.getX(DistanceUnit.INCH), targetPose.getY(DistanceUnit.INCH), targetPose.getHeading(AngleUnit.DEGREES));

            myOpMode.telemetry.addData("Target Position", data);
            myOpMode.telemetry.addData("XReached", xController.targetReached);
            myOpMode.telemetry.addData("YReached", yController.targetReached);
            myOpMode.telemetry.addData("HReached", headingController.targetReached);
            myOpMode.telemetry.addData("targetReached", targetReached);
            myOpMode.telemetry.addData("xPower", xPower);
            myOpMode.telemetry.addData("xPowerRotated", xPower_rotated);
        } else if (drivetrainMode == DrivetrainMode.BACKUP){

        }


    }

    // (Other drivetrain methods remain unchanged.)
}
