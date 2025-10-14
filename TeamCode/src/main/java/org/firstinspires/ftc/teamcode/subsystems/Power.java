package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.PIDController;

public class Power {
    private LinearOpMode myOpMode = null;
    public DcMotor leftBase = null;
    public DcMotor rightBase = null;

    PIDController baseLeftPID;
    PIDController baseRightPID;

    public static final double BASE_KP = 0.005;
    public static final double BASE_KI = 0;
    public static final double BASE_KD = 0.0;

    public Power(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {

        baseLeftPID = new PIDController(BASE_KP, BASE_KI, BASE_KD, 0.9);
        baseRightPID = new PIDController(BASE_KP, BASE_KI, BASE_KD, 0.9);

        baseLeftPID.maxOut = 0.9;
        baseRightPID.maxOut = 0.9;

        rightBase = myOpMode.hardwareMap.get(DcMotor.class, "rightBase");
        leftBase = myOpMode.hardwareMap.get(DcMotor.class, "leftBase");

        leftBase.setDirection(DcMotor.Direction.REVERSE);
        rightBase.setDirection(DcMotor.Direction.FORWARD);

        // brake and encoders
        rightBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    public void teleOp(){
        if(myOpMode.gamepad2.dpad_up){
            baseToPositionPIDClass(2000);
        } else {
            baseToPositionPIDClass(0);
        }
    }

    public void update(){

    }

    public void baseToPositionPIDClass(double targetPosition) {
        double outLeft = baseLeftPID.calculate(targetPosition, leftBase.getCurrentPosition());
        double outRight = baseRightPID.calculate(targetPosition, rightBase.getCurrentPosition());

        leftBase.setPower(outLeft);
        rightBase.setPower(outRight);

        myOpMode.telemetry.addData("lift", leftBase.getCurrentPosition());
        myOpMode.telemetry.addData("lift", rightBase.getCurrentPosition());

        myOpMode.telemetry.addData("LiftLeftPower: ", outLeft);
        myOpMode.telemetry.addData("LiftRightPower: ", outRight);
    }


}
