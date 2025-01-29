package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Extension {
    private LinearOpMode myOpMode = null;

    public CRServo leftLink = null;
    public CRServo rightLink = null;

    public static final double LEFT_LINK_IN = 0.2;
    public static final double RIGHT_LINK_IN = 0.8; //used to be at 0
    public static final double LEFT_LINK_OUT = 0.4;
    public static final double RIGHT_LINK_OUT = 0.6; //used to be 0.4

    public double leftLinkPosition;
    public double rightLinkPosition;

    public Extension(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        leftLink = myOpMode.hardwareMap.get(CRServoImplEx.class, "leftLink");
        rightLink = myOpMode.hardwareMap.get(CRServoImplEx.class, "rightLink");

        leftLink.setPower(0);
        rightLink.setPower(0);

        leftLinkPosition = LEFT_LINK_IN;
        rightLinkPosition = RIGHT_LINK_IN;
    }

    public void teleOp() {
        /*leftLink.setPower(leftLinkPosition);
        rightLink.setPosition(rightLinkPosition);*/
        if(Math.abs(myOpMode.gamepad2.right_stick_x) > 0.1){
            /*leftLinkPosition = LEFT_LINK_IN;
            rightLinkPosition = RIGHT_LINK_IN;*/
            leftLink.setPower(-myOpMode.gamepad2.right_stick_x); //
            rightLink.setPower(myOpMode.gamepad2.right_stick_x);
        } else {
            leftLink.setPower(-0.06);
            rightLink.setPower(0.06);
        }

    }

}
