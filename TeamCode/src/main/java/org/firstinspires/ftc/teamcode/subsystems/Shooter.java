package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    private LinearOpMode myOpMode = null;
    public DcMotorEx shootingMotor = null;
    public Servo linkageShooting = null;
    public CRServoImplEx transferServo = null;

    public final static double LOWER_THRESHOLD_MOTOR = 1000;
    public final static double LOWER_THRESHOLD_TRANSFER = 0.5;
    public final static int UPPER_THRESHOLD = 0;
    public final static int CLOSE_SHOOTER_SPEED = 0;
    public final static int FAR_SHOOTER_SPEED = 0;

    public int TICKS_PER_SECOND = 0;
    public int TICKS_PER_REVOLUTION = 28;
    public int REVOLUTIONS_PER_MINUTE = 6000;

    public double LINKAGE_UP = 0.5;
    public double LINKAGE_DOWN = 0.1;

    public  double TRANSFER_SPEED = 0.8;

    ElapsedTime timer = new ElapsedTime();

    public Shooter (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init (){
        shootingMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "shooterMotor");
        linkageShooting  = myOpMode.hardwareMap.get(Servo.class, "linkageServo");
        transferServo = myOpMode.hardwareMap.get(CRServoImplEx.class, "transferServo");
        timer.reset();

        linkageShooting.setPosition(LINKAGE_DOWN);

    }

    public void teleOp(){

        //calculate Ticks per second based on current RPM
        TICKS_PER_SECOND = REVOLUTIONS_PER_MINUTE/60*TICKS_PER_REVOLUTION;

        //calculate measured RPM from motors current degrees per second
        double measuredRPM = shootingMotor.getVelocity()/TICKS_PER_REVOLUTION*60;

        if(myOpMode.gamepad2.right_bumper){
            shootingMotor.setVelocity(0);
        }else if(myOpMode.gamepad2.left_bumper){
            shootingMotor.setVelocity(-TICKS_PER_SECOND);
        }

        if (myOpMode.gamepad2.dpad_up) {
            linkageShooting.setPosition(LINKAGE_UP);
            transferServo.setPower(TRANSFER_SPEED);
        } else if (myOpMode.gamepad2.dpad_down){
            linkageShooting.setPosition(LINKAGE_DOWN);
            transferServo.setPower(0);
        }

    }

    public Action shooterAction(int velocity) {
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    shootingMotor.setVelocity(velocity);
                    initialized = true;
                }
                return shootingMotor.getVelocity() > LOWER_THRESHOLD_MOTOR;
            }
        };
    }

    public Action linkageAction(double power) {
        timer.reset();
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    linkageShooting.setPosition(LINKAGE_UP);
                    initialized = true;
                }
                return timer.seconds() > 0.5;
            }
        };
    }

    public Action transferAction(double power) {
        timer.reset();

        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    transferServo.setPower(power);
                    initialized = true;
                }
                return transferServo.getPower() > LOWER_THRESHOLD_TRANSFER;
            }
        };
    }
    public void update(){

    }
}
