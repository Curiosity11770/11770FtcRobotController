package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.PIDController;

public class Spindexer {
    private LinearOpMode myOpMode = null;

    public CRServoImplEx spindexerServo = null;
    public AnalogInput spindexerEncoder;

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

    public static final double[] LOAD_POSITIONS = {1,2,3};

    public static final double SPINDEXER_KP = 0.3;
    public static final double SPINDEXER_KI = 0;
    public static final double SPINDEXER_KD = 0.0;


    public final static int LOWER_THRESHOLD = 0;

    public enum SpindexerMode {
        MANUAL,
        INTAKE
    }

    public SpindexerMode spindexerMode = SpindexerMode.INTAKE;
    public int spindexerTargetIndex = 0;
    public double spindexerTargetPosition = LOAD_POSITIONS[spindexerTargetIndex];


    ElapsedTime timer = new ElapsedTime();

    public Spindexer (LinearOpMode opmode){
        myOpMode = opmode;
    }

    public void init (){
        spindexerServo = myOpMode.hardwareMap.get(CRServoImplEx.class, "spindexerServo");
        spindexerEncoder = myOpMode.hardwareMap.get(AnalogInput.class, "analogInput");

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

        if (spindexerMode == SpindexerMode.MANUAL) {
            if (myOpMode.gamepad2.a) {
                spindexerServo.setPower(0.2);
            } else if (myOpMode.gamepad2.b) {
                spindexerServo.setPower(-0.2);
            } else if (myOpMode.gamepad2.x) {
                spindexerServo.setPower(0.1);
            } else {
                spindexerServo.setPower(0);
            }
        }else if(spindexerMode == SpindexerMode.INTAKE) {
            spindexerToPositionPIDClass(spindexerTargetPosition);

            //if color sensor detects artifact
            if (hsvValuesOne[0] > 60) {
                //advance desired position
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

    public Action spindexerAction(double power) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    spindexerServo.setPower(power);
                    initialized = true;
                }
                return spindexerServo.getPower() > LOWER_THRESHOLD;
            }
        };
    }

    public void update(){

    }

    public void spindexerToPositionPIDClass(double targetPosition) {
        double output = spindexerPID.calculate(targetPosition, spindexerEncoder.getVoltage());

        spindexerServo.setPower(output);

        myOpMode.telemetry.addData("spindexer", spindexerEncoder.getVoltage());
        myOpMode.telemetry.addData("output", output);
    }

}
