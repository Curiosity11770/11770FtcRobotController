package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Intake {
    private LinearOpMode myOpMode = null;
    public DcMotor spinIntake = null;
    public Servo flipIntake = null;

    public static final double INTAKE_UP = 0.34;
    public static final double INTAKE_DOWN = 0.6;
    public static final double INTAKE_STOWED = 0;

    public boolean transfer = false;

    public double flipPosition;

    public NormalizedColorSensor colorSensor;
    final float[] hsvValues = new float[3];
    public NormalizedRGBA colors;

    public enum IntakeMode {
        TRANSFER,
        STOWED,
        INTAKE
    }
    public IntakeMode intakeMode = IntakeMode.STOWED;


    public Intake(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){


        spinIntake = myOpMode.hardwareMap.get(DcMotor.class, "spinIntake");
        flipIntake = myOpMode.hardwareMap.get(Servo.class, "flipIntake");

        colorSensor = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }



        spinIntake.setPower(0);
        myOpMode.telemetry.addData("intake", flipIntake.getPosition());
        flipPosition = INTAKE_STOWED;
        flipIntake.setPosition(INTAKE_STOWED);

    }

    public void sensorUpdate(){
        colorSensor.setGain(2);
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        myOpMode.telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        myOpMode.telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        myOpMode.telemetry.addData("Alpha", "%.3f", colors.alpha);
        myOpMode.telemetry.addData("Gain", 2);
        /* If this color sensor also has a distance sensor, display the measured distance.
         * Note that the reported distance is only useful at very close range, and is impacted by
         * ambient light and surface reflectivity. */
        if (colorSensor instanceof DistanceSensor) {
            myOpMode.telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        }
    }

    public void teleOp(){

        sensorUpdate();
        update();

        if(myOpMode.gamepad2.b){
            intakeMode = IntakeMode.TRANSFER;
        }else if(myOpMode.gamepad2.a){
            intakeMode = IntakeMode.INTAKE;
        } else if (myOpMode.gamepad2.dpad_left){
            intakeMode = IntakeMode.TRANSFER;
            spinIntake.setPower(1);
        }

        /*
        if(myOpMode.gamepad2.a) {
            intakeMode = IntakeMode.INTAKE;
        } else if (myOpMode.gamepad2.b){
            intakeMode = IntakeMode.TRANSFER;
        }
        */


        if(myOpMode.gamepad2.left_trigger > 0.7) {
            spinIntake.setPower(1);
            transfer = false;
        } else if (myOpMode.gamepad2.right_trigger > 0.7){
            spinIntake.setPower(-1);
            transfer = false;
        } else if (myOpMode.gamepad2.dpad_left) {
            spinIntake.setPower(1);
            transfer = true;
        /*} else if (transfer == true){
            spinIntake.setPower(1);*/
        } else {
            spinIntake.setPower(0);
        }
    }

    public void update(){
        flipIntake.setPosition(flipPosition);
        if (intakeMode == IntakeMode.INTAKE){
            flipPosition = INTAKE_DOWN;
        } else if (intakeMode == IntakeMode.TRANSFER){
            spinIntake.setPower(0);
            flipPosition = INTAKE_UP;
        } else if(intakeMode == IntakeMode.STOWED){
            flipPosition = INTAKE_UP;
        }
    }

}
