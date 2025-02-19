package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Extension {
    private LinearOpMode myOpMode = null;

    public Servo leftLink = null;
    public Servo rightLink = null;

    public static final double LEFT_LINK_IN = 0.8;//used to be at 0
    public static final double LEFT_LINK_OUT = 0.4;//used to be 0.4

    public double leftLinkPosition = LEFT_LINK_IN;
    public double rightLinkPosition;

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
        leftLink.setPosition(LEFT_LINK_IN);
        rightLink.setPosition(LEFT_LINK_IN);

    }

    public void teleOp() {
        update();
        leftLink.setPosition(leftLinkPosition);
        if(Math.abs(myOpMode.gamepad2.left_stick_y) > 0.1){
            extensionMode = ExtensionMode.MANUAL;
        } else if (myOpMode.gamepad2.dpad_left){
            extensionMode = ExtensionMode.TRANSFER;
        }
    }
    public void update() {
        if(extensionMode == ExtensionMode.MANUAL) {
            myOpMode.telemetry.addData("extension", leftLinkPosition);
        }
        if(Math.abs(myOpMode.gamepad2.left_stick_y) > 0.1){
            leftLinkPosition = 1-(-myOpMode.gamepad2.left_stick_y*0.8);
        } else if (extensionMode == ExtensionMode.TRANSFER){
            leftLinkPosition = 0.95;
        }

    }

}