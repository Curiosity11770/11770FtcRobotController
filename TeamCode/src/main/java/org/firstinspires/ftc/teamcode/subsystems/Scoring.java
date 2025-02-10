package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring {

    private LinearOpMode myOpMode = null;
    public Servo clawWrist = null;
    public Servo clawServo = null;
    public Servo scoringPivot = null;

    public static final double CLAW_UP = 0.88;
    public static final double CLAW_DOWN = 0.2;
    public static final double CLAW_OPEN = 0.8;
    public static final double CLAW_CLOSED = 0.32;
    public static final double CHAMBER_SCORING = 0;
    public static final double TRANSFER_SCORING = 0.45;
    public static final double WALL_SCORING = 0.6;

    public enum ScoringMode {
        SCORING_HIGH_CHAMBER,
        PICKUP_WALL,
        TRANSFER

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
    }

    public void update(){
        myOpMode.telemetry.addData("pivotPosition", scoringPivot.getPosition());
        myOpMode.telemetry.addData("scoringMode", scoringMode);

        if (scoringMode == ScoringMode.TRANSFER) {
            scoringPivot.setPosition(TRANSFER_SCORING);
        } else if (scoringMode == ScoringMode.SCORING_HIGH_CHAMBER){
            scoringPivot.setPosition(CHAMBER_SCORING);
        } else if (scoringMode == ScoringMode.PICKUP_WALL){
            scoringPivot.setPosition(WALL_SCORING);
        }

    }

    public void teleOp() {
        update();
        //position = scoringPivot.getVoltage();
        if (myOpMode.gamepad2.left_trigger > 0.7) {
            clawServo.setPosition(CLAW_OPEN);
        } else if (myOpMode.gamepad2.right_trigger > 0.7) {
            clawServo.setPosition(CLAW_CLOSED);
        }
        if(myOpMode.gamepad2.x){
            clawWrist.setPosition(CLAW_UP);
        } else if (myOpMode.gamepad2.y){
            clawWrist.setPosition(CLAW_DOWN);;
        }
        if (myOpMode.gamepad2.dpad_up){
            scoringMode = scoringMode.SCORING_HIGH_CHAMBER;
        }else if(myOpMode.gamepad2.dpad_down){
            scoringMode = scoringMode.PICKUP_WALL;
        } else if (myOpMode.gamepad2.dpad_left){
            scoringMode = scoringMode.TRANSFER;
        }

    }
}