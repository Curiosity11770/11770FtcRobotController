package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.PIDController;
@Config

public class Spindexer {
    private LinearOpMode myOpMode = null;

    public CRServoImplEx spindexerServo = null;
    public AnalogInput spindexerEncoder;
    public Servo rgb0 = null;
    public Servo rgb1 = null;
    public Servo rgb2 = null;
    public DigitalChannel revGreen0 = null;
    public DigitalChannel revRed0 = null;
    public DigitalChannel revGreen1 = null;
    public DigitalChannel revRed1 = null;
    public DigitalChannel revGreen2 = null;
    public DigitalChannel revRed2 = null;

    public RevColorSensorV3 colorSensorOne;
    public final float[] hsvValuesOne = new float[3];
    public NormalizedRGBA colorsOne;
    public NormalizedColorSensor colorSensorTwo;
    public final float[] hsvValuesTwo = new float[3];
    public NormalizedRGBA colorsTwo;
    public NormalizedColorSensor colorSensorThree;
    public final float[] hsvValuesThree = new float[3];
    public NormalizedRGBA colorsThree;

    PIDController spindexerPID;

    private DigitalChannel laserInput;

    public enum ColorMode {
        PURPLE,
        EMPTY,
        GREEN
    }

    public static final double[] LOAD_POSITIONS = {0.62,1.72,2.82};

    public ColorMode[] COLOR_STATUS = {ColorMode.EMPTY,ColorMode.EMPTY,ColorMode.EMPTY};

    public ColorMode[] MOTIF_ORDER = {ColorMode.GREEN,ColorMode.PURPLE,ColorMode.PURPLE};

    public static final double SPINDEXER_KP = 0.3;
    public static final double SPINDEXER_KI = 0;
    public static final double SPINDEXER_KD = 0.0;


    public final static int LOWER_THRESHOLD = 0;

    public static double SPINDEXER_SPEED = 0.65;

    public boolean isTriggered = false;

    public int THRESHOLD = 100;
    public enum SpindexerMode {
        CONTINUOUS,
        AUTO_INTAKE,
        MANUAL_INTAKE,
        SORTING
    }

    public boolean CHECK = true;

    public SpindexerMode spindexerMode = SpindexerMode.CONTINUOUS;
    public int spindexerTargetIndex = 0;
    public double spindexerTargetPosition = LOAD_POSITIONS[spindexerTargetIndex];


    ElapsedTime timer = new ElapsedTime();

    Intake intake;
    Shooter shooter;

    public Spindexer (LinearOpMode opmode, Intake myIntake, Shooter myShooter){
        myOpMode = opmode;
        intake = myIntake;
        shooter = myShooter;
    }

    public void init (){
        spindexerServo = myOpMode.hardwareMap.get(CRServoImplEx.class, "spindexerServo");
        spindexerEncoder = myOpMode.hardwareMap.get(AnalogInput.class, "analogInput");

        rgb0 = myOpMode.hardwareMap.get(Servo.class, "rgb0");
        rgb1 = myOpMode.hardwareMap.get(Servo.class, "rgb1");
        rgb2 = myOpMode.hardwareMap.get(Servo.class, "rgb2");

        revGreen0 = myOpMode.hardwareMap.get(DigitalChannel.class, "revGreen0");
        revRed0 = myOpMode.hardwareMap.get(DigitalChannel.class, "revRed0");
        revGreen1 = myOpMode.hardwareMap.get(DigitalChannel.class, "revGreen1");
        revRed1 = myOpMode.hardwareMap.get(DigitalChannel.class, "revRed1");
        revGreen2 = myOpMode.hardwareMap.get(DigitalChannel.class, "revGreen2");
        revRed2 = myOpMode.hardwareMap.get(DigitalChannel.class, "revRed2");


        colorSensorOne = myOpMode.hardwareMap.get(RevColorSensorV3.class, "colorSensorOne");
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

        revGreen0.setMode(DigitalChannel.Mode.OUTPUT);
        revRed0.setMode(DigitalChannel.Mode.OUTPUT);

        revGreen1.setMode(DigitalChannel.Mode.OUTPUT);
        revRed1.setMode(DigitalChannel.Mode.OUTPUT);
        revGreen2.setMode(DigitalChannel.Mode.OUTPUT);
        revRed2.setMode(DigitalChannel.Mode.OUTPUT);

        laserInput = myOpMode.hardwareMap.get(DigitalChannel.class, "laserDigitalInput");


        laserInput.setMode(DigitalChannel.Mode.INPUT);


    }
    public void teleOp() {
        colorSensorOne.setGain(2);
        colorsOne = colorSensorOne.getNormalizedColors();
        Color.colorToHSV(colorsOne.toColor(), hsvValuesOne);

        colorSensorTwo.setGain(2);
        colorsTwo = colorSensorTwo.getNormalizedColors();
        Color.colorToHSV(colorsTwo.toColor(), hsvValuesTwo);

        colorSensorThree.setGain(2);
        colorsThree = colorSensorThree.getNormalizedColors();
        Color.colorToHSV(colorsThree.toColor(), hsvValuesThree);

        /*myOpMode.telemetry.addLine()
                .addData("Red", "%.3f", colorsOne.red)
                .addData("Green", "%.3f", colorsOne.green)
                .addData("Blue", "%.3f", colorsOne.blue);
        myOpMode.telemetry.addLine()
                .addData("Hue", "%.3f", hsvValuesOne[0])
                .addData("Saturation", "%.3f", hsvValuesOne[1])
                .addData("Value", "%.3f", hsvValuesOne[2]);
        myOpMode.telemetry.addData("Alpha", "%.3f", colorsOne.alpha);

        myOpMode.telemetry.addLine()
                .addData("Red", "%.3f", colorsTwo.red)
                .addData("Green", "%.3f", colorsTwo.green)
                .addData("Blue", "%.3f", colorsTwo.blue);
        myOpMode.telemetry.addLine()
                .addData("Hue", "%.3f", hsvValuesTwo[0])
                .addData("Saturation", "%.3f", hsvValuesTwo[1])
                .addData("Value", "%.3f", hsvValuesTwo[2]);
        myOpMode.telemetry.addData("Alpha", "%.3f", colorsTwo.alpha);

        myOpMode.telemetry.addLine()
                .addData("Red", "%.3f", colorsThree.red)
                .addData("Green", "%.3f", colorsThree.green)
                .addData("Blue", "%.3f", colorsThree.blue);
        myOpMode.telemetry.addLine()
                .addData("Hue", "%.3f", hsvValuesThree[0])
                .addData("Saturation", "%.3f", hsvValuesThree[1])
                .addData("Value", "%.3f", hsvValuesThree[2]);
        myOpMode.telemetry.addData("Alpha", "%.3f", colorsThree.alpha);*/

        myOpMode.telemetry.addData("colorStatus", COLOR_STATUS[0]);
        myOpMode.telemetry.addData("colorStatus", COLOR_STATUS[1]);
        myOpMode.telemetry.addData("colorStatus", COLOR_STATUS[2]);


        boolean detected = laserInput.getState();


        // Display detection state
        if (detected) {
            myOpMode.telemetry.addLine("Object detected!");
        } else {
            myOpMode.telemetry.addLine("No object detected");
        }

        if (laserInput.getState()) {
           /* COLOR_STATUS[0] = ColorMode.EMPTY;
            COLOR_STATUS[1] = ColorMode.EMPTY;
            COLOR_STATUS[2] = ColorMode.EMPTY;*/
        }




        /*
        colorChanger(rgb0, 0);
        colorChanger(rgb1, 1);
        colorChanger(rgb2, 2);

        colorRev(hsvValuesOne, revGreen0, revRed0);
        colorRev(hsvValuesTwo, revGreen1, revRed1);
        colorRev(hsvValuesThree, revGreen2, revRed2);
        */

        colorUpdate(hsvValuesOne, rgb0);
        colorUpdate(hsvValuesTwo, rgb1);
        colorUpdate(hsvValuesThree, rgb2);

        if (myOpMode.gamepad2.a) {
            spindexerMode = SpindexerMode.CONTINUOUS;
        } else if (myOpMode.gamepad2.b){
            spindexerMode = SpindexerMode.CONTINUOUS;
        } else if (myOpMode.gamepad2.dpad_left){
            spindexerTargetIndex = 0;
            spindexerTargetPosition = LOAD_POSITIONS[spindexerTargetIndex];
            isTriggered = false;
            COLOR_STATUS[0] = ColorMode.EMPTY;
            COLOR_STATUS[1] = ColorMode.EMPTY;
            COLOR_STATUS[2] = ColorMode.EMPTY;

            spindexerMode = SpindexerMode.AUTO_INTAKE;
        } else if (myOpMode.gamepad2.dpad_right){
            spindexerMode = SpindexerMode.MANUAL_INTAKE;
        } else if(myOpMode.gamepad2.x){
            spindexerMode = SpindexerMode.SORTING;
        } else if (myOpMode.gamepad2.y){
            spindexerMode = SpindexerMode.SORTING;
        }

        if (spindexerMode == SpindexerMode.CONTINUOUS) {
            if (myOpMode.gamepad2.a) {
                if(shooter.transferOn) {
                    if (Math.abs(shooter.REVOLUTIONS_PER_MINUTE - Math.abs(shooter.measuredRPM)) < THRESHOLD) {
                        spindexerServo.setPower(SPINDEXER_SPEED);
                    } else {
                        spindexerServo.setPower(0);
                    }
                } else {
                    spindexerServo.setPower(SPINDEXER_SPEED);
            }
            } else if (myOpMode.gamepad2.b) {
                spindexerServo.setPower(-SPINDEXER_SPEED);
            } else {
                spindexerServo.setPower(0);
            }
        }else if(spindexerMode == SpindexerMode.AUTO_INTAKE) {
            spindexerToPositionPIDClass(spindexerTargetPosition);

            //if color sensor detects artifact
            if (colorSensorOne.getDistance(DistanceUnit.MM) < 50 && !isTriggered && spindexerTargetIndex < 2) {

                if (hsvValuesOne[0] > 200){
                    COLOR_STATUS[spindexerTargetIndex] = ColorMode.PURPLE;
                } else if (hsvValuesOne[0] > 130){
                    COLOR_STATUS[spindexerTargetIndex] = ColorMode.GREEN;
                } else if (hsvValuesOne[0] < 80){
                    COLOR_STATUS[spindexerTargetIndex] = ColorMode.EMPTY;
                }

                //advance desired position
                spindexerTargetIndex = (spindexerTargetIndex + 1) % 3;
                spindexerTargetPosition = LOAD_POSITIONS[spindexerTargetIndex];
                isTriggered = true;
            } else if (colorSensorOne.getDistance(DistanceUnit.MM) < 50 && !isTriggered && spindexerTargetIndex == 2){
                if (hsvValuesOne[0] > 200){
                    COLOR_STATUS[spindexerTargetIndex] = ColorMode.PURPLE;
                } else if (hsvValuesOne[0] > 130){
                    COLOR_STATUS[spindexerTargetIndex] = ColorMode.GREEN;
                } else if (hsvValuesOne[0] < 70){
                    COLOR_STATUS[spindexerTargetIndex] = ColorMode.EMPTY;
                }
                if(!(myOpMode.gamepad2.right_trigger > 0.2) &&  !(myOpMode.gamepad2.left_trigger > 0.2)) {
                    intake.intakeMotor.setPower(0.5);
                }
            }

            if(Math.abs(spindexerTargetPosition- spindexerEncoder.getVoltage()) < 0.2){
                isTriggered = false;
            }

        } else if(spindexerMode == SpindexerMode.MANUAL_INTAKE) {
            spindexerToPositionPIDClass(spindexerTargetPosition);
            if (myOpMode.gamepad2.dpad_right) {
                spindexerTargetIndex = (spindexerTargetIndex + 1) % 3;
                spindexerTargetPosition = LOAD_POSITIONS[spindexerTargetIndex];
            }
        } else if (spindexerMode == SpindexerMode.SORTING){
            if(myOpMode.gamepad2.y){
                if(hsvValuesThree[0] > 200) {
                    if (CHECK){
                        timer.reset();
                }
                    CHECK = false;
                    spindexerServo.setPower(SPINDEXER_SPEED);
                    shooter.transferOn = true;
                    shooter.transferServo.setPower(shooter.TRANSFER_SPEED);
                    shooter.linkageShooting.setPosition(shooter.LINKAGE_UP);
                } else {
                    spindexerServo.setPower(0.1);
                    CHECK = true;
                    if(timer.seconds() > 1.5 || laserInput.getState()) {
                        shooter.transferOn = false;
                        shooter.transferServo.setPower(0);
                        shooter.linkageShooting.setPosition(shooter.LINKAGE_DOWN);
                    }
                }
            } else if (myOpMode.gamepad2.x){
                if(hsvValuesThree[0] < 200 && hsvValuesThree[0] > 100){
                    if (CHECK){
                        timer.reset();
                    }
                    CHECK = false;
                    spindexerServo.setPower(SPINDEXER_SPEED);
                    shooter.transferOn = true;
                    shooter.transferServo.setPower(shooter.TRANSFER_SPEED);
                    shooter.linkageShooting.setPosition(shooter.LINKAGE_UP);
                } else {
                    spindexerServo.setPower(0.1);
                    CHECK = true;
                    if(timer.seconds() > 1.5 || laserInput.getState()) {
                        shooter.transferOn = false;
                        shooter.transferServo.setPower(0);
                        shooter.linkageShooting.setPosition(shooter.LINKAGE_DOWN);
                    }
                }

            }
        }

        myOpMode.telemetry.addData("Spindexer Mode: ", spindexerMode);
        myOpMode.telemetry.addData("Target Index: ", spindexerTargetIndex);
        //myOpMode.telemetry.addData("Target Position: ", spindexerTargetPosition);
        myOpMode.telemetry.addData("Current Position", spindexerEncoder.getVoltage());

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("spindexerSpeed", SPINDEXER_SPEED);
        dashboardTelemetry.update();
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

                myOpMode.telemetry.addData("spindexerAction", actionTimer.seconds());
                myOpMode.telemetry.update();
                return actionTimer.seconds() < time;
            }
        };
    }

    public Action shootingMotif(int id) {
        ElapsedTime actionTimer = new ElapsedTime();
        actionTimer.reset();
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    actionTimer.reset();
                    initialized = true;
                }

                if (id == 21){
                    spindexerToPositionPIDClass(LOAD_POSITIONS[0]);
                } else if (id == 22){
                    spindexerToPositionPIDClass(LOAD_POSITIONS[1]);

                }  else if (id == 23){
                    spindexerToPositionPIDClass(LOAD_POSITIONS[2]);

                }
                myOpMode.telemetry.addData("ColorStatus: ", COLOR_STATUS[0]);
                myOpMode.telemetry.addData("ColorStatus1", COLOR_STATUS[1]);
                myOpMode.telemetry.addData("ColorStatus2: ", COLOR_STATUS[2]);

                myOpMode.telemetry.addData("Motif Order",MOTIF_ORDER[0]);
                myOpMode.telemetry.addData("Motif Order2",MOTIF_ORDER[1]);
                myOpMode.telemetry.addData("Motif Order3",MOTIF_ORDER[2]);
                myOpMode.telemetry.update();
                return actionTimer.seconds() < 0.5;
            }
        };
    }

    public Action shootingMotif3(int id) {
        ElapsedTime actionTimer = new ElapsedTime();
        actionTimer.reset();
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    actionTimer.reset();
                    initialized = true;
                }

                if (id == 21){
                    spindexerToPositionPIDClass(LOAD_POSITIONS[2]);
                } else if (id == 22){
                    spindexerToPositionPIDClass(LOAD_POSITIONS[0]);

                }  else if (id == 23){
                    spindexerToPositionPIDClass(LOAD_POSITIONS[1]);

                }
                myOpMode.telemetry.addData("ColorStatus: ", COLOR_STATUS[0]);
                myOpMode.telemetry.addData("ColorStatus1", COLOR_STATUS[1]);
                myOpMode.telemetry.addData("ColorStatus2: ", COLOR_STATUS[2]);

                myOpMode.telemetry.addData("Motif Order",MOTIF_ORDER[0]);
                myOpMode.telemetry.addData("Motif Order2",MOTIF_ORDER[1]);
                myOpMode.telemetry.addData("Motif Order3",MOTIF_ORDER[2]);
                myOpMode.telemetry.update();
                return actionTimer.seconds() < 0.5;
            }
        };
    }
    public Action shootingMotif2(int id) {
        ElapsedTime actionTimer = new ElapsedTime();
        actionTimer.reset();
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    actionTimer.reset();
                    initialized = true;
                }
                spindexerToPositionPIDClass(spindexerTargetPosition);

                if (id == 21){
                    spindexerToPositionPIDClass(LOAD_POSITIONS[1]);
                } else if (id == 22){
                    spindexerToPositionPIDClass(LOAD_POSITIONS[2]);

                }  else if (id == 23){
                    spindexerToPositionPIDClass(LOAD_POSITIONS[0]);

                }
                myOpMode.telemetry.addData("ColorStatus: ", COLOR_STATUS[0]);
                myOpMode.telemetry.addData("ColorStatus1", COLOR_STATUS[1]);
                myOpMode.telemetry.addData("ColorStatus2: ", COLOR_STATUS[2]);

                myOpMode.telemetry.addData("Motif Order",MOTIF_ORDER[0]);
                myOpMode.telemetry.addData("Motif Order2",MOTIF_ORDER[1]);
                myOpMode.telemetry.addData("Motif Order3",MOTIF_ORDER[2]);
                myOpMode.telemetry.update();
                return actionTimer.seconds() < 0.5;
            }
        };
    }


    public Action setColorStatus(ColorMode color1, ColorMode color2, ColorMode color3) {
        ElapsedTime actionTimer = new ElapsedTime();
        actionTimer.reset();
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    actionTimer.reset();
                    initialized = true;
                }
                COLOR_STATUS[0] = color1;
                COLOR_STATUS[1] = color2;
                COLOR_STATUS[2] = color3;
                myOpMode.telemetry.addData("ColorStatus: ", COLOR_STATUS[0]);
                myOpMode.telemetry.addData("ColorStatus1", COLOR_STATUS[1]);
                myOpMode.telemetry.addData("ColorStatus2: ", COLOR_STATUS[2]);
                myOpMode.telemetry.update();
                return actionTimer.seconds() < 0.01;
            }
        };
    }

    public Action setMotifOrder(int id) {
        ElapsedTime actionTimer = new ElapsedTime();
        actionTimer.reset();
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    actionTimer.reset();
                    initialized = true;
                }
                if (id == 21){
                    MOTIF_ORDER = new ColorMode[]{ColorMode.GREEN, ColorMode.PURPLE, ColorMode.PURPLE};

                }  else if (id == 22){

                    MOTIF_ORDER = new ColorMode[]{ColorMode.PURPLE, ColorMode.GREEN, ColorMode.PURPLE};
                } else if (id == 23) {

                    MOTIF_ORDER = new ColorMode[]{ColorMode.PURPLE, ColorMode.PURPLE, ColorMode.GREEN};
                }
                myOpMode.telemetry.addData("Motif Order",MOTIF_ORDER[0]);
                myOpMode.telemetry.addData("Motif Order2",MOTIF_ORDER[1]);
                myOpMode.telemetry.addData("Motif Order3",MOTIF_ORDER[2]);
                myOpMode.telemetry.update();
                return actionTimer.seconds() < 0.01;
            }
        };
    }

    public Action autoIntake() {
        ElapsedTime actionTimer = new ElapsedTime();
        actionTimer.reset();
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    actionTimer.reset();
                    spindexerTargetIndex = 0;
                    spindexerTargetPosition = LOAD_POSITIONS[spindexerTargetIndex];
                    isTriggered = false;
                    spindexerMode = SpindexerMode.AUTO_INTAKE;
                    initialized = true;
                }


                colorSensorOne.setGain(2);
                colorsOne = colorSensorOne.getNormalizedColors();
                Color.colorToHSV(colorsOne.toColor(), hsvValuesOne);

                colorSensorTwo.setGain(2);
                colorsTwo= colorSensorTwo.getNormalizedColors();
                Color.colorToHSV(colorsTwo.toColor(), hsvValuesTwo);

                colorSensorThree.setGain(2);
                colorsThree = colorSensorThree.getNormalizedColors();
                Color.colorToHSV(colorsThree.toColor(), hsvValuesThree);


                myOpMode.telemetry.addLine()
                        .addData("Red", "%.3f", colorsOne.red)
                        .addData("Green", "%.3f", colorsOne.green)
                        .addData("Blue", "%.3f", colorsOne.blue);
                myOpMode.telemetry.addLine()
                        .addData("Hue", "%.3f", hsvValuesOne[0])
                        .addData("Saturation", "%.3f", hsvValuesOne[1])
                        .addData("Value", "%.3f", hsvValuesOne[2]);
                myOpMode.telemetry.addData("Alpha", "%.3f", colorsOne.alpha);

                myOpMode.telemetry.addLine()
                        .addData("Red", "%.3f", colorsTwo.red)
                        .addData("Green", "%.3f", colorsTwo.green)
                        .addData("Blue", "%.3f", colorsTwo.blue);
                myOpMode.telemetry.addLine()
                        .addData("Hue", "%.3f", hsvValuesTwo[0])
                        .addData("Saturation", "%.3f", hsvValuesTwo[1])
                        .addData("Value", "%.3f", hsvValuesTwo[2]);
                myOpMode.telemetry.addData("Alpha", "%.3f", colorsTwo.alpha);

                myOpMode.telemetry.addLine()
                        .addData("Red", "%.3f", colorsThree.red)
                        .addData("Green", "%.3f", colorsThree.green)
                        .addData("Blue", "%.3f", colorsThree.blue);
                myOpMode.telemetry.addLine()
                        .addData("Hue", "%.3f", hsvValuesThree[0])
                        .addData("Saturation", "%.3f", hsvValuesThree[1])
                        .addData("Value", "%.3f", hsvValuesThree[2]);
                myOpMode.telemetry.addData("Alpha", "%.3f", colorsThree.alpha);

                colorChanger(rgb0, 0);
                colorChanger(rgb1, 1);
                colorChanger(rgb2, 2);

                spindexerToPositionPIDClass(spindexerTargetPosition);

                //if color sensor detects artifact
                if (colorSensorOne.getDistance(DistanceUnit.MM) < 50 && !isTriggered && spindexerTargetIndex < 2) {
                    //advance desired position
                    if (hsvValuesOne[0] > 200){
                        COLOR_STATUS[spindexerTargetIndex] = ColorMode.PURPLE;
                    } else if (hsvValuesOne[0] > 100){
                        COLOR_STATUS[spindexerTargetIndex] = ColorMode.GREEN;
                    } else if (hsvValuesOne[0] < 20){
                        COLOR_STATUS[spindexerTargetIndex] = ColorMode.EMPTY;
                    }
                    spindexerTargetIndex = (spindexerTargetIndex + 1) % 3;
                    spindexerTargetPosition = LOAD_POSITIONS[spindexerTargetIndex];
                    isTriggered = true;
                } else if (colorSensorOne.getDistance(DistanceUnit.MM) < 50 && !isTriggered && spindexerTargetIndex == 2){
                    if (hsvValuesOne[0] > 200){
                        COLOR_STATUS[spindexerTargetIndex] = ColorMode.PURPLE;
                    } else if (hsvValuesOne[0] > 100){
                        COLOR_STATUS[spindexerTargetIndex] = ColorMode.GREEN;
                    } else if (hsvValuesOne[0] < 20){
                        COLOR_STATUS[spindexerTargetIndex] = ColorMode.EMPTY;
                    }

                    intake.intakeMotor.setPower(1);
                }

                if(Math.abs(spindexerTargetPosition- spindexerEncoder.getVoltage()) < 0.2){
                    isTriggered = false;
                }


                myOpMode.telemetry.addData("Spindexer Mode: ", spindexerMode);
                myOpMode.telemetry.addData("Target Index: ", spindexerTargetIndex);
                myOpMode.telemetry.addData("Target Position: ", spindexerTargetPosition);
                myOpMode.telemetry.addData("Current Position", spindexerEncoder.getVoltage());
                myOpMode.telemetry.update();

                return actionTimer.seconds() < 3;
            }
        };
    }



    public void update(){

    }

    public void colorUpdate(float [] hsvValues, Servo servo){
        if (hsvValues[0] > 200){
            servo.setPosition(0.722);
        } else if (hsvValues[0] > 100) {
            servo.setPosition(0.500);
        } else if (hsvValues[0] < 60){
            servo.setPosition(0);
        }

    }

    public void colorRev(float [] hsvValues, DigitalChannel servo, DigitalChannel servo2){
        if (hsvValues[0] > 200){
            servo.setState(false);
            servo2.setState(true);
            myOpMode.telemetry.addData("red ", servo2);
        } else if (hsvValues[0] > 100) {
            servo.setState(true);
            servo2.setState(false);
        } else if (hsvValues[0] < 60){
            servo.setState(true);
            servo2.setState(true);
        }

    }


    public void colorChanger(Servo servo, int position){
        if (COLOR_STATUS[position] == ColorMode.PURPLE) {
            servo.setPosition(0.722);
        } else if (COLOR_STATUS[position] == ColorMode.GREEN){
            servo.setPosition(0.500);
        } else {
            servo.setPosition(0);
        }
    }

    public void spindexerToPositionPIDClass(double targetPosition) {
        double output = spindexerPID.calculate(targetPosition, spindexerEncoder.getVoltage());

        if(Math.abs(targetPosition - spindexerEncoder.getVoltage()) > 0.2) {
            spindexerServo.setPower(-output);
        } else {
            spindexerServo.setPower(0);
        }

        myOpMode.telemetry.addData("spindexer", spindexerEncoder.getVoltage());
        myOpMode.telemetry.addData("output", output);
    }

}
