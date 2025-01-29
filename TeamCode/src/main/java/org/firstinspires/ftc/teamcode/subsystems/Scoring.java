package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring {

    private LinearOpMode myOpMode = null;
    public Servo clawWrist = null;
    public Servo clawServo = null;
    public Servo scoringPivot = null;

    public static final double CLAW_UP = 0.82;
    public static final double CLAW_DOWN = 0.142;
    public static final double CLAW_OPEN = 0.59;
    public static final double CLAW_CLOSED = 0.41;
    public static final double SCORING_UP = 0.78;
    public static final double SCORING_DOWN = 0.31;

    public double clawPosition;
    public double clawRotation;
    public double scoringPosition;

    public double testPosition;
    public AnalogInput scoringTest;

    public Scoring(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        scoringPivot = myOpMode.hardwareMap.get(Servo.class, "scoringPivot");
        clawWrist = myOpMode.hardwareMap.get(Servo.class, "clawWrist");
        clawServo = myOpMode.hardwareMap.get(Servo.class, "clawServo");
        scoringTest = myOpMode.hardwareMap.get(AnalogInput.class, "scoringTest");

        testPosition = scoringTest.getVoltage() / 3.3 * 360;
        //scoringPivot.setPower(0);
                //analogInput.getVoltage() / 3.3 * 360;
        clawWrist.setPosition(CLAW_DOWN);
        clawServo.setPosition(CLAW_CLOSED);
        clawPosition = CLAW_CLOSED;
        clawRotation = CLAW_DOWN;
        scoringPosition = SCORING_DOWN;
    }

    public void teleOp() {
        clawWrist.setPosition(clawRotation);
        clawServo.setPosition(clawPosition);
        //scoringPivot.setPosition(scoringPosition);
        //position = scoringPivot.getVoltage();
        if (myOpMode.gamepad2.left_bumper) {
            clawPosition = CLAW_OPEN;
        } else if (myOpMode.gamepad2.right_bumper) {
            clawPosition = CLAW_CLOSED;
        }
        if(myOpMode.gamepad2.x){
            clawRotation = CLAW_UP;
        } else if (myOpMode.gamepad2.y){
            clawRotation = CLAW_DOWN;
        }
        if (myOpMode.gamepad2.dpad_up){
            scoringPivot.setPosition(SCORING_UP);
        }else if(myOpMode.gamepad2.dpad_down){
            scoringPivot.setPosition(SCORING_DOWN);
        } else if (myOpMode.gamepad2.dpad_right){
            scoringPivot.setPosition(0);
        }else if(myOpMode.gamepad2.dpad_left){
            scoringPivot.setPosition(.98);
        }
        /*if(testPosition >= 300){
            scoringPivot.setPower(0);
        }*/

        myOpMode.telemetry.addData("Position", testPosition);

    }
}
