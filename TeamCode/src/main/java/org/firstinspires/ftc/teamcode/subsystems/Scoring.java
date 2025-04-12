package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SampleAuto;

public class Scoring {

    private LinearOpMode myOpMode = null;
    public Servo clawWrist = null;
    public Servo clawServo = null;
    public Servo scoringPivot = null;

    public static final double CLAW_UP = 0.87;
    public static final double CLAW_DOWN = 0.19;
    public static final double CLAW_OPEN = 0.4;
    public static final double CLAW_CLOSED = 0.15;
    public static final double CHAMBER_SCORING = 0.19;
    public static final double TRANSFER_SCORING = 0.325;
    public static final double WALL_SCORING = 0.62;
    public static final double SAMPLE_SCORING = 0.83;
    public double clawServoPosition;
    public double clawWristPosition;
    public ElapsedTime transferTimer = new ElapsedTime();

    public enum ScoringMode {
        SCORING_HIGH_CHAMBER,
        PICKUP_WALL,
        TRANSFER,
        SAMPLE

    }

    public ScoringMode scoringMode = ScoringMode.TRANSFER;

    public Scoring(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        scoringPivot = myOpMode.hardwareMap.get(Servo.class, "scoringPivot");
        clawWrist = myOpMode.hardwareMap.get(Servo.class, "clawWrist");
        clawServo = myOpMode.hardwareMap.get(Servo.class, "clawServo");
        clawWrist.setPosition(CLAW_UP);
        clawServo.setPosition(CLAW_CLOSED);

        scoringPivot.setPosition(TRANSFER_SCORING);

        clawServoPosition = CLAW_CLOSED;
        clawWristPosition = CLAW_UP;
    }

    public void update(){
        myOpMode.telemetry.addData("pivotPosition", scoringPivot.getPosition());
        myOpMode.telemetry.addData("scoringMode", scoringMode);

        clawServo.setPosition(clawServoPosition);
        clawWrist.setPosition(clawWristPosition);

        if (scoringMode == ScoringMode.TRANSFER) {
            scoringPivot.setPosition(TRANSFER_SCORING);
            //clawServoPosition = CLAW_OPEN;
            if (transferTimer.seconds() > 2){
                //clawServoPosition = CLAW_CLOSED;
            }
        } else if (scoringMode == ScoringMode.SCORING_HIGH_CHAMBER){
            scoringPivot.setPosition(CHAMBER_SCORING);
        } else if (scoringMode == ScoringMode.PICKUP_WALL){
            scoringPivot.setPosition(WALL_SCORING);
        } else if (scoringMode == ScoringMode.SAMPLE){
            scoringPivot.setPosition(SAMPLE_SCORING);
        }

    }

    public void teleOp() {
        update();
        //position = scoringPivot.getVoltage();
        if (myOpMode.gamepad2.left_bumper) {
            clawServoPosition = CLAW_OPEN;
        } else if (myOpMode.gamepad2.right_bumper) {
            clawServoPosition = CLAW_CLOSED;
        } else if (myOpMode.gamepad2.dpad_left){
            clawServoPosition = CLAW_OPEN;
        }

        if(myOpMode.gamepad2.x){
            clawWristPosition = CLAW_UP;
        } else if (myOpMode.gamepad2.y){
            clawWristPosition = CLAW_DOWN;
        }

        if (myOpMode.gamepad2.dpad_up){
            scoringMode = ScoringMode.SCORING_HIGH_CHAMBER;
        }else if(myOpMode.gamepad2.dpad_down){
            scoringMode = ScoringMode.PICKUP_WALL;
        } else if (myOpMode.gamepad2.dpad_left) {
            scoringMode = ScoringMode.TRANSFER;
            transferTimer.reset();
        } else if (myOpMode.gamepad2.dpad_right){
            scoringMode = ScoringMode.SAMPLE;
        }

    }
}