package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.PIDController;

public class Lift {
    private LinearOpMode myOpMode = null;

    //lift motors
    public DcMotor rightLift = null;
    public DcMotor leftLift = null;
    public ServoImplEx rightHook = null;
    public ServoImplEx  leftHook = null;

    public static final double RIGHT_HOOK_UP = 0.03;
    public static final double LEFT_HOOK_UP = 0.32;
    public static final double RIGHT_HOOK_DOWN = 0.27;
    public static final double LEFT_HOOK_DOWN  = 0.08;    //touch sensor
    //public TouchSensor touch = null;

    public enum LiftMode {
        MANUAL,
        HIGH_CHAMBER,
        HIGH_BASKET,
        GROUND,
        LOW_BASKET,
        LOW_CHAMBER
    }

    public LiftMode liftMode = LiftMode.MANUAL;
    PIDController liftLeftPID;
    PIDController liftRightPID;

    public static final double LIFT_KP = 0.005;
    public static final double LIFT_KI = 0;
    public static final double LIFT_KD = 0.0;

    public Lift(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {

        liftLeftPID = new PIDController(LIFT_KP, LIFT_KI, LIFT_KD, 0.9);
        liftRightPID = new PIDController(LIFT_KP, LIFT_KI, LIFT_KD, 0.9);
        liftLeftPID.maxOut = 0.9;
        liftRightPID.maxOut = 0.9;


        rightLift = myOpMode.hardwareMap.get(DcMotor.class, "rightLift");
        leftLift = myOpMode.hardwareMap.get(DcMotor.class, "leftLift");

        rightHook = myOpMode.hardwareMap.get(ServoImplEx.class, "rightHook");
        leftHook = myOpMode.hardwareMap.get(ServoImplEx.class, "leftHook");

        leftHook.setPosition(LEFT_HOOK_UP);
        rightHook.setPosition(RIGHT_HOOK_UP);

        //touch = myOpMode.hardwareMap.get(TouchSensor.class, "touch");

        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);

        // brake and encoders
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        myOpMode.telemetry.addData(">", "Lift Initialized");
    }

    public void teleOp() {
        //gamepad control specific to lift
        //if (Math.abs(myOpMode.gamepad2.right_stick_y) > 0.8) {
          //  liftMode = LiftMode.MANUAL;
        //}
        if(myOpMode.gamepad1.x){
            rightHook.setPosition(RIGHT_HOOK_UP);
            leftHook.setPosition(LEFT_HOOK_UP);
        } else if (myOpMode.gamepad1.y){
            rightHook.setPosition(RIGHT_HOOK_DOWN);
            leftHook.setPosition(LEFT_HOOK_DOWN);
        }

        if (myOpMode.gamepad1.dpad_up){
            leftHook.setPwmEnable();
            rightHook.setPwmEnable();
        } else if (myOpMode.gamepad1.dpad_down){
            leftHook.setPwmDisable();
            rightHook.setPwmDisable();
        }

       myOpMode.telemetry.addData("enabled", rightHook.isPwmEnabled());
        myOpMode.telemetry.addData("enabled", leftHook.isPwmEnabled());

        if(myOpMode.gamepad2.dpad_up){
           // liftToPositionPIDClass(100);
        } else if (myOpMode.gamepad2.dpad_right){
           // liftToPositionPIDClass(1300);
        }

        //code defining behavior of lift in each state
            if (Math.abs(myOpMode.gamepad2.left_stick_y) > 0.1) {
                liftMode = liftMode.MANUAL;
                leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //robot.liftLeft.setPower(-0.8);
                //robot.liftRight.setPower(-0.8);
                rightLift.setPower(-myOpMode.gamepad2.left_stick_y); //
                leftLift.setPower(-myOpMode.gamepad2.left_stick_y); //
            } else {
                leftLift.setPower(0.07);
                rightLift.setPower(0.07);
            }

        myOpMode.telemetry.addData("left_stick_y: ", -myOpMode.gamepad2.left_stick_y);
        myOpMode.telemetry.addData("Lift Mode ", liftMode);

    }

    public void update() {
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (liftMode == LiftMode.HIGH_CHAMBER) {
            liftToPositionPIDClass(1300);
        } else if (liftMode == LiftMode.HIGH_BASKET){
            liftToPositionPIDClass(2600);
        } else if (liftMode == LiftMode.GROUND){
            liftToPositionPIDClass(0);
        }
        myOpMode.telemetry.addData("lift", leftLift.getCurrentPosition());
        myOpMode.telemetry.addData("lift", rightLift.getCurrentPosition());
    }

    public void liftToPositionPIDClass(double targetPosition) {
        double outLeft = liftLeftPID.calculate(targetPosition, leftLift.getCurrentPosition());
        double outRight = liftRightPID.calculate(targetPosition, rightLift.getCurrentPosition());

        leftLift.setPower(outLeft);
        rightLift.setPower(outRight);

        myOpMode.telemetry.addData("lift", leftLift.getCurrentPosition());
        myOpMode.telemetry.addData("lift", rightLift.getCurrentPosition());

        myOpMode.telemetry.addData("LiftLeftPower: ", outLeft);
        myOpMode.telemetry.addData("LiftRightPower: ", outRight);
    }


}