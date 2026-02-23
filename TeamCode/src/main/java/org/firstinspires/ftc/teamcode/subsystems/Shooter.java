package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public class Shooter {
    private LinearOpMode myOpMode = null;
    public DcMotorEx shootingMotor = null;
    public Servo linkageShooting = null;
    public CRServoImplEx transferServo = null;

    public Vision vision;

    public final static double LOWER_THRESHOLD_MOTOR = 1000;
    public final static double LOWER_THRESHOLD_TRANSFER = 0.5;
    public final static int UPPER_THRESHOLD = 0;
    public final static int CLOSE_SHOOTER_SPEED = 0;
    public final static int FAR_SHOOTER_SPEED = 0;

    public double TICKS_PER_SECOND = 0;
    public int TICKS_PER_REVOLUTION = 28;
    public double REVOLUTIONS_PER_MINUTE = 4200;

    public double LINKAGE_UP = 0.3;
    public double LINKAGE_DOWN = 0.72    ;

    public double TRANSFER_SPEED = -1;

    ElapsedTime timer = new ElapsedTime();

    private int velocity = 500; // starting velocity (ticks per second)
    private final int VELOCITY_INCREMENT = 100; // how much to change per button press
    private final int MAX_VELOCITY = 6000; // limit to avoid over-speeding
    private final int MIN_VELOCITY = 0;

    private boolean aPressedLast = false;
    private boolean bPressedLast = false;

    List<LLResultTypes.FiducialResult> fiducialResults;


    public Shooter (LinearOpMode opmode, Vision robotVision) {
        myOpMode = opmode;
        vision = robotVision;
    }

    public Shooter (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init (){
        shootingMotor  = myOpMode.hardwareMap.get(DcMotorEx.class, "shooterMotor");
        linkageShooting  = myOpMode.hardwareMap.get(Servo.class, "linkageServo");
        transferServo = myOpMode.hardwareMap.get(CRServoImplEx.class, "transferServo");


        linkageShooting.setPosition(LINKAGE_DOWN);

    }

    public void teleOp(){
        if (myOpMode.gamepad1.left_trigger > 0.2 || myOpMode.gamepad1.right_trigger > 0.2) {
            fiducialResults = vision.result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if(fr.getTargetPoseCameraSpace().getPosition().z <  2.5) {
                    REVOLUTIONS_PER_MINUTE = 1044 * fr.getTargetPoseCameraSpace().getPosition().z + 1967;
                    TICKS_PER_SECOND = REVOLUTIONS_PER_MINUTE / 60 * TICKS_PER_REVOLUTION;
                    shootingMotor.setVelocity(-TICKS_PER_SECOND);
                } else if (fr.getTargetPoseCameraSpace().getPosition().z > 2.5){
                    REVOLUTIONS_PER_MINUTE = 1244 * fr.getTargetPoseCameraSpace().getPosition().z + 1967;
                    TICKS_PER_SECOND = REVOLUTIONS_PER_MINUTE / 60 * TICKS_PER_REVOLUTION;
                    shootingMotor.setVelocity(-TICKS_PER_SECOND);
                }

            }

        } else {
            REVOLUTIONS_PER_MINUTE = 3500;
        }
        //calculate Ticks per second based on current RPM
        TICKS_PER_SECOND = REVOLUTIONS_PER_MINUTE/60*TICKS_PER_REVOLUTION;

        if(myOpMode.gamepad2.right_bumper){
            shootingMotor.setVelocity(0);
        }else if(myOpMode.gamepad2.left_bumper){
            shootingMotor.setVelocity(-TICKS_PER_SECOND);
        }

        /*if (myOpMode.gamepad1.a && !aPressedLast) {
            velocity += VELOCITY_INCREMENT;
            if (velocity > MAX_VELOCITY) velocity = MAX_VELOCITY;
            REVOLUTIONS_PER_MINUTE = velocity;
            TICKS_PER_SECOND = REVOLUTIONS_PER_MINUTE/60*TICKS_PER_REVOLUTION;
            shootingMotor.setVelocity(-TICKS_PER_SECOND);
        }

        // Button B: Decrease velocity
        if (myOpMode.gamepad1.b && !bPressedLast) {
            velocity -= VELOCITY_INCREMENT;
            if (velocity < MIN_VELOCITY) velocity = MIN_VELOCITY;
            REVOLUTIONS_PER_MINUTE = velocity;
            TICKS_PER_SECOND = REVOLUTIONS_PER_MINUTE/60*TICKS_PER_REVOLUTION;
            shootingMotor.setVelocity(-TICKS_PER_SECOND);
        }

        // Update flags
        aPressedLast = myOpMode.gamepad1.a;
        bPressedLast = myOpMode.gamepad1.b;*/


        if (myOpMode.gamepad2.dpad_up) {
            linkageShooting.setPosition(LINKAGE_UP);
            transferServo.setPower(TRANSFER_SPEED);
        } else if (myOpMode.gamepad2.dpad_down){
            linkageShooting.setPosition(LINKAGE_DOWN);
            transferServo.setPower(0);
        }


        myOpMode.telemetry.addData("shooter rpm", REVOLUTIONS_PER_MINUTE);

    }

    public Action shooterAction(double velocity, double time) {
        ElapsedTime actionTimer = new ElapsedTime();
        actionTimer.reset();
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    actionTimer.reset();
                    shootingMotor.setVelocity(-velocity);
                    initialized = true;
                }
                return actionTimer.seconds() < time;
            }
        };
    }

    public Action linkageAction(double linkageAction, double time) {
        ElapsedTime actionTimer = new ElapsedTime();
        actionTimer.reset();
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    actionTimer.reset();
                    linkageShooting.setPosition(linkageAction);
                    initialized = true;
                }
                return actionTimer.seconds() < time;
            }
        };
    }

    public Action linkageOff(double time) {
        ElapsedTime actionTimer = new ElapsedTime();
        actionTimer.reset();
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    actionTimer.reset();
                    linkageShooting.setPosition(LINKAGE_DOWN);
                    transferServo.setPower(0);
                    initialized = true;
                }
                return actionTimer.seconds() < time;
            }
        };
    }

    public Action transferAction(double power, double time) {
        ElapsedTime actionTimer = new ElapsedTime();
        actionTimer.reset();
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    actionTimer.reset();
                    transferServo.setPower(power);
                    initialized = true;
                }
                return actionTimer.seconds()  < time;
            }
        };
    }


    public void update(){

    }
}
