package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring {
    private LinearOpMode myOpMode = null;

    public Servo leftArmServo = null;

    public Servo rightArmServo = null;

    public Servo boxServo = null;

    public Servo leftGateServo;
    public Servo rightGateServo;

    public boolean is_open_left;

    public boolean is_open_right;

    public boolean isUp;

    public Scoring(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    public void init() {
        leftArmServo = myOpMode.hardwareMap.get(Servo.class, "armServoLeft");
        rightArmServo = myOpMode.hardwareMap.get(Servo.class, "armServoRight");
        boxServo = myOpMode.hardwareMap.get(Servo.class, "boxServo");
        leftGateServo = myOpMode.hardwareMap.get(Servo.class, "gateServoLeft");
        rightGateServo = myOpMode.hardwareMap.get(Servo.class, "gateServoRight");

        leftArmServo.setPosition(0.08);
        rightArmServo.setPosition(0.92);
        boxServo.setPosition(0.94);
        leftGateServo.setPosition(0);
        rightGateServo.setPosition(0.7);

        is_open_left = false;
        is_open_right = false;

        isUp = false;


    }

    public void teleOp(boolean firstBreak, boolean secondBreak) {

        if (myOpMode.gamepad2.x) {
            leftGateServo.setPosition(0.5);
            rightGateServo.setPosition(0);
            is_open_left = true;
        }
        //myOpMode.telemetry.addData("isWorking");
        if (myOpMode.gamepad2.y) {
            rightGateServo.setPosition(0.7);
            leftGateServo.setPosition(0);
            is_open_right = true;
        /*} else if (secondBreak){
            rightGateServo.setPosition(0);
            is_open_right = true;*/
        }
        if(myOpMode.gamepad2.dpad_up) {
            leftArmServo.setPosition(0.3);
            rightArmServo.setPosition(0.7);
            isUp = true;
        }
        if(myOpMode.gamepad2.dpad_down) {
            leftArmServo.setPosition(0.08);
            rightArmServo.setPosition(0.92);
            isUp = false;
        }

        if(myOpMode.gamepad2.right_bumper) {
            boxServo.setPosition(0.94);
        } else if(myOpMode.gamepad2.left_bumper) {
            if(isUp = true) {
                boxServo.setPosition(0.58);
            }
        }
    }


    public void deliver(double power, double time) {
        
    }

}
