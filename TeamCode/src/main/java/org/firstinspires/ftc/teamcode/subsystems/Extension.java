package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Extension {
    private LinearOpMode myOpMode = null;

    public Servo leftLink = null;
    public Servo rightLink = null;

    public static final double LEFT_LINK_IN = 0.89;//used to be at 0
    public static final double RIGHT_LINK_IN = 0.89;//used to be 0.4

    public double leftLinkPosition = LEFT_LINK_IN;
    public double rightLinkPosition = RIGHT_LINK_IN;

    public enum ExtensionMode {
        MANUAL,
        TRANSFER

    }
    public Extension.ExtensionMode extensionMode = Extension.ExtensionMode.MANUAL;

    public Extension(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        leftLink = myOpMode.hardwareMap.get(Servo.class, "leftLink");
        rightLink = myOpMode.hardwareMap.get(Servo.class, "rightLink");

        leftLinkPosition = LEFT_LINK_IN;
        rightLinkPosition = RIGHT_LINK_IN;
        leftLink.setPosition(LEFT_LINK_IN);
        rightLink.setPosition(RIGHT_LINK_IN);

    }

    public void teleOp() {
        update();
        if(Math.abs(myOpMode.gamepad2.left_stick_y) > 0.1){
            extensionMode = ExtensionMode.MANUAL;
        } else if (myOpMode.gamepad2.dpad_left){
            extensionMode = ExtensionMode.TRANSFER;
        }
    }
    public void update() {
        leftLink.setPosition(leftLinkPosition);
        rightLink.setPosition(rightLinkPosition);
        if(extensionMode == ExtensionMode.MANUAL) {
            myOpMode.telemetry.addData("extensionLeft", leftLinkPosition);
            myOpMode.telemetry.addData("extensionRight", rightLinkPosition);
            myOpMode.telemetry.addData("extensionSum", rightLinkPosition + leftLinkPosition);
            if (Math.abs(myOpMode.gamepad2.left_stick_y) > 0.1) {
                leftLinkPosition += 0.05 * myOpMode.gamepad2.left_stick_y;
                rightLinkPosition += 0.05 * myOpMode.gamepad2.left_stick_y;
            }
            if (leftLinkPosition > 0.89) {
                leftLinkPosition = 0.89;
            } else if (leftLinkPosition < 0) {
                leftLinkPosition = 0;
            }

            if (rightLinkPosition > 0.89) {
                rightLinkPosition = 0.89;
            } else if (rightLinkPosition < 0) {
                rightLinkPosition = 0;
            }
        } else if (extensionMode == ExtensionMode.TRANSFER){
            leftLinkPosition = 0.89;
            rightLinkPosition = 0.89;

        }

    }

}