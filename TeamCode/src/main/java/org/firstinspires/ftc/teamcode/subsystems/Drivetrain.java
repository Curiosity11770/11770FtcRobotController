package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.PinPointLocalizer;

import java.util.Locale;
@Config
public class Drivetrain {
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    ElapsedTime time = new ElapsedTime();

    //private ElapsedTime timeout = null;

    public HuskyLens huskylens;

    //drivetrain
    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;

    public PinPointLocalizer localizer;

    PIDController xController;
    PIDController yController;
    PIDController headingController;

    public boolean targetReached = false;
    Pose2D targetPose;

    //Static Variables

    public static double HEADING_KP = 0.011;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.0;
    public static double DRIVE_KP = 0.025;
    public static double DRIVE_KI = 0.0;
    public static double DRIVE_KD = 0.0;//0.0003;
    public static double DRIVE_MAX_ACC = 2000;
    public static double DRIVE_MAX_VEL = 3500;
    public static double DRIVE_MAX_OUT = 0.8;
    public static double STRAFE_MULTIPLIER = 2.0;

    public MecanumDrive mecanumDrive = null;

    public Drivetrain(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){

        targetPose = new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.DEGREES,0);

        xController = new PIDController(DRIVE_KP,DRIVE_KI,DRIVE_KD, DRIVE_MAX_OUT);
        yController = new PIDController(DRIVE_KP,DRIVE_KI,DRIVE_KD,DRIVE_MAX_OUT);
        headingController = new PIDController(HEADING_KP,HEADING_KI,HEADING_KD,DRIVE_MAX_OUT);

        localizer = new PinPointLocalizer(myOpMode);

        localizer.init();

        time.reset();
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

        huskylens = myOpMode.hardwareMap.get(HuskyLens.class, "huskyLens");
        huskylens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

    }
    public void teleOp() {
        localizer.update();
        //drive train
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        double drive = -myOpMode.gamepad1.left_stick_y;
        double turn = myOpMode.gamepad1.right_stick_x;
        double strafe = -myOpMode.gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 2);

        frontLeftPower = (drive + turn - strafe) / denominator;
        frontRightPower = (drive - turn + strafe) / denominator;
        backLeftPower = (drive + turn + strafe) / denominator;
        backRightPower = (drive - turn - strafe) / denominator;

        if (myOpMode.gamepad1.right_trigger > 0.3) {
            leftFrontDrive.setPower(frontLeftPower / 7);
            rightFrontDrive.setPower(frontRightPower / 7);
            leftBackDrive.setPower(backLeftPower / 7);
            rightBackDrive.setPower(backRightPower / 7);
        } else if (myOpMode.gamepad1.left_trigger > 0.3) {
            leftFrontDrive.setPower(2 * frontLeftPower);
            rightFrontDrive.setPower(2 * frontRightPower);
            leftBackDrive.setPower(2 * backLeftPower);
            rightBackDrive.setPower(2 * backRightPower);
        } else {
            leftBackDrive.setPower(backLeftPower);
            leftFrontDrive.setPower(frontLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);
        }
    }

    public void driveTime(double speed, double seconds) {
        time.reset();
        while(myOpMode.opModeIsActive() && seconds < time.seconds()){
            leftBackDrive.setPower(speed);
            leftFrontDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            rightBackDrive.setPower(speed);
        }
    }
    public void strafeTime(double speed, double seconds) {
        time.reset();
        while(myOpMode.opModeIsActive() && seconds < time.seconds()){
            leftBackDrive.setPower(-speed);
            leftFrontDrive.setPower(speed);
            rightFrontDrive.setPower(-speed);
            rightBackDrive.setPower(speed);
        }
    }

    public void update(){

        localizer.update();

        //Use PIDs to calculate motor powers based on error to targets
        double xPower = xController.calculate(targetPose.getX(DistanceUnit.INCH), localizer.getX());
        double yPower = yController.calculate(targetPose.getY(DistanceUnit.INCH), localizer.getY());

        double wrappedAngleError = angleWrap(targetPose.getHeading(AngleUnit.DEGREES) - localizer.getHeading());
        double tPower = headingController.calculate(wrappedAngleError);

        double radianHeading = Math.toRadians(localizer.getHeading());

        //rotate the motor powers based on robot heading
        double xPower_rotated = xPower * Math.cos(-radianHeading) - yPower * Math.sin(-radianHeading);
        double yPower_rotated = xPower * Math.sin(-radianHeading) + yPower * Math.cos(-radianHeading);

        // x, y, theta input mixing to deliver motor powers
        leftFrontDrive.setPower(xPower_rotated - yPower_rotated * STRAFE_MULTIPLIER - tPower);
        leftBackDrive.setPower(xPower_rotated + yPower_rotated * STRAFE_MULTIPLIER - tPower);
        rightFrontDrive.setPower(xPower_rotated + yPower_rotated * STRAFE_MULTIPLIER + tPower);
        rightBackDrive.setPower(xPower_rotated - yPower_rotated * STRAFE_MULTIPLIER + tPower);

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
    }

    public void setTargetPose(Pose2D newTarget){
        targetPose = newTarget;
        targetReached = false;
    }

    public void driveToPose(double xTarget, double yTarget, double degreeTarget) {
        //check if drivetrain is still working towards target
        targetReached = xController.targetReached && yController.targetReached && headingController.targetReached;
        //double thetaTarget = Math.toRadians(degreeTarget);
        //Use PIDs to calculate motor powers based on error to targets
        double xPower = xController.calculate(xTarget, localizer.getX());
        double yPower = yController.calculate(yTarget, localizer.getY());

        double wrappedAngleError = angleWrap(degreeTarget - localizer.getHeading());
        double tPower = headingController.calculate(wrappedAngleError);

        double radianHeading = Math.toRadians(localizer.getHeading());

        //rotate the motor powers based on robot heading
        double xPower_rotated = xPower * Math.cos(-radianHeading) - yPower * Math.sin(-radianHeading);
        double yPower_rotated = xPower * Math.sin(-radianHeading) + yPower * Math.cos(-radianHeading);

        // x, y, theta input mixing to deliver motor powers
        leftFrontDrive.setPower(xPower_rotated - yPower_rotated - tPower);
        leftBackDrive.setPower(xPower_rotated + yPower_rotated - tPower);
        rightFrontDrive.setPower(xPower_rotated + yPower_rotated + tPower);
        rightBackDrive.setPower(xPower_rotated - yPower_rotated + tPower);
    }

    public void stop(){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    // This function normalizes the angle so it returns a value between -180° and 180° instead of 0° to 360°.
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
