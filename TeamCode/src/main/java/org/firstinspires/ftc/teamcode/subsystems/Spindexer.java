package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.PIDController;

public class Spindexer {
    private LinearOpMode myOpMode = null;

    public CRServoImplEx spindexerServo = null;
    public AnalogInput spindexerEncoder;
    public Servo rgb0 = null;
    public Servo rgb1 = null;
    public Servo rgb2 = null;

    public NormalizedColorSensor colorSensorOne;
    public final float[] hsvValuesOne = new float[3];
    public NormalizedRGBA colorsOne;
    public NormalizedColorSensor colorSensorTwo;
    public final float[] hsvValuesTwo = new float[3];
    public NormalizedRGBA colorsTwo;
    public NormalizedColorSensor colorSensorThree;
    public final float[] hsvValuesThree = new float[3];
    public NormalizedRGBA colorsThree;

    PIDController spindexerPID;

    //Spindexer Constants

    public enum ColorMode {
        PURPLE,
        EMPTY,
        GREEN
    }

    public static final double[] LOAD_POSITIONS = {0.1,1.1,2.2};
    public static final ColorMode[] COLOR_STATUS = {ColorMode.EMPTY,ColorMode.EMPTY,ColorMode.EMPTY};


    public static final double SPINDEXER_KP = 0.3;
    public static final double SPINDEXER_KI = 0;
    public static final double SPINDEXER_KD = 0.0;


    public final static int LOWER_THRESHOLD = 0;

    public boolean isTriggered = false;

    public enum SpindexerMode {
        CONTINUOUS,
        AUTO_INTAKE,
        MANUAL_INTAKE
    }

    public SpindexerMode spindexerMode = SpindexerMode.CONTINUOUS;
    public int spindexerTargetIndex = 0;
    public double spindexerTargetPosition = LOAD_POSITIONS[spindexerTargetIndex];


    ElapsedTime timer = new ElapsedTime();

    public Spindexer (LinearOpMode opmode){
        myOpMode = opmode;
    }

    public void init (){
        spindexerServo = myOpMode.hardwareMap.get(CRServoImplEx.class, "spindexerServo");
        spindexerEncoder = myOpMode.hardwareMap.get(AnalogInput.class, "analogInput");

        rgb0 = myOpMode.hardwareMap.get(Servo.class, "rgb0");
        rgb1 = myOpMode.hardwareMap.get(Servo.class, "rgb1");
        rgb2 = myOpMode.hardwareMap.get(Servo.class, "rgb2");

        colorSensorOne = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensorOne");
        colorSensorTwo = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensorTwo");
        colorSensorThree = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensorThree");

        if (colorSensorOne instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorOne).enableLight(true);
        }

        if (colorSensorTwo instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorTwo).enableLight(true);
        }

        if (colorSensorThree instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorThree).enableLight(true);
        }

        spindexerPID = new PIDController(SPINDEXER_KP, SPINDEXER_KI, SPINDEXER_KD, 0.9);

        timer.reset();

    }
    public void teleOp() {
        colorSensorOne.setGain(2);
        colorsOne = colorSensorOne.getNormalizedColors();
        Color.colorToHSV(colorsOne.toColor(), hsvValuesOne);

        myOpMode.telemetry.addLine()
                .addData("Red", "%.3f", colorsOne.red)
                .addData("Green", "%.3f", colorsOne.green)
                .addData("Blue", "%.3f", colorsOne.blue);
        myOpMode.telemetry.addLine()
                .addData("Hue", "%.3f", hsvValuesOne[0])
                .addData("Saturation", "%.3f", hsvValuesOne[1])
                .addData("Value", "%.3f", hsvValuesOne[2]);
        myOpMode.telemetry.addData("Alpha", "%.3f", colorsOne.alpha);

        colorSensorTwo.setGain(2);
        colorsTwo = colorSensorTwo.getNormalizedColors();
        Color.colorToHSV(colorsTwo.toColor(), hsvValuesTwo);

        colorSensorThree.setGain(2);
        colorsThree = colorSensorThree.getNormalizedColors();
        Color.colorToHSV(colorsThree.toColor(), hsvValuesThree);

        colorChanger(rgb0, 0);
        colorChanger(rgb1, 1);
        colorChanger(rgb2, 2);

        if(myOpMode.gamepad2.x){
            spindexerMode = SpindexerMode.CONTINUOUS;
        } else if (myOpMode.gamepad2.dpad_left){
            spindexerTargetIndex = 0;
            spindexerTargetPosition = LOAD_POSITIONS[spindexerTargetIndex];
            isTriggered = false;
            spindexerMode = SpindexerMode.AUTO_INTAKE;
        } else if (myOpMode.gamepad2.dpad_right){
            spindexerMode = SpindexerMode.MANUAL_INTAKE;
        }

        if (spindexerMode == SpindexerMode.CONTINUOUS) {
            if (myOpMode.gamepad2.a) {
                spindexerServo.setPower(0.2);
            } else if (myOpMode.gamepad2.b) {
                spindexerServo.setPower(-0.2);
            } else if (myOpMode.gamepad2.x) {
                spindexerServo.setPower(0.1);
            } else {
                spindexerServo.setPower(0);
            }
        }else if(spindexerMode == SpindexerMode.AUTO_INTAKE) {
            spindexerToPositionPIDClass(spindexerTargetPosition);

            //if color sensor detects artifact
            if (hsvValuesOne[0] > 60 && !isTriggered && spindexerTargetIndex < 2) {
                //advance desired position
                spindexerTargetIndex = (spindexerTargetIndex + 1) % 3;
                spindexerTargetPosition = LOAD_POSITIONS[spindexerTargetIndex];
                isTriggered = true;
            }

            if(Math.abs(spindexerTargetPosition- spindexerEncoder.getVoltage()) < 0.2){
                isTriggered = false;
            }

            if (hsvValuesOne[0] > 200){
                COLOR_STATUS[spindexerTargetIndex] = ColorMode.PURPLE;
            } else if (hsvValuesOne[0] > 100){
                COLOR_STATUS[spindexerTargetIndex] = ColorMode.GREEN;
            } else if (hsvValuesOne[0] < 20){
                COLOR_STATUS[spindexerTargetIndex] = ColorMode.EMPTY;
            }
        } else if(spindexerMode == SpindexerMode.MANUAL_INTAKE) {
            spindexerToPositionPIDClass(spindexerTargetPosition);
            if (myOpMode.gamepad2.dpad_right) {
                spindexerTargetIndex = (spindexerTargetIndex + 1) % 3;
                spindexerTargetPosition = LOAD_POSITIONS[spindexerTargetIndex];
            }
        }

        myOpMode.telemetry.addData("Spindexer Mode: ", spindexerMode);
        myOpMode.telemetry.addData("Target Index: ", spindexerTargetIndex);
        myOpMode.telemetry.addData("Target Position: ", spindexerTargetPosition);
        myOpMode.telemetry.addData("Current Position", spindexerEncoder.getVoltage());
        //add telemetry for color value
    }

    public Action spindexerAction(double power, double time) {
        ElapsedTime actionTimer = new ElapsedTime();
        actionTimer.reset();
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    actionTimer.reset();
                    spindexerServo.setPower(power);
                    initialized = true;
                }
                return actionTimer.seconds() < time;
            }
        };
    }

    public void update(){

    }

    public void colorChanger(Servo servo, int position){
        if (COLOR_STATUS[position] == ColorMode.PURPLE) {
            servo.setPosition(0.722);
        } else if (COLOR_STATUS[position] == ColorMode.GREEN){
            servo.setPosition(0.500);
        } else {
            servo.setPosition(0.99);
        }
    }

    public void spindexerToPositionPIDClass(double targetPosition) {
        double output = spindexerPID.calculate(targetPosition, spindexerEncoder.getVoltage());

        spindexerServo.setPower(-output);

        myOpMode.telemetry.addData("spindexer", spindexerEncoder.getVoltage());
        myOpMode.telemetry.addData("output", output);
    }

}
