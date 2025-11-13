package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

// The AnalogInput class is what we use to read an analog sensor.
// In this case, it's a simple value read from a potentiometer or other sensor.
// The sample code is designed to read a voltage, which you can use to control other aspects of the robot.

@TeleOp(name="AnalogTest", group="Linear OpMode")
public class AnalogTest extends LinearOpMode {

    private AnalogInput analogInput;

    private CRServo servo;


    @Override
    public void runOpMode() {
        // You will need to declare and initialize your AnalogInput.
        // This is done by getting the hardware map and assigning it to a variable.
        servo = hardwareMap.get(CRServoImplEx.class, "spindexerServo");
        analogInput = hardwareMap.get(AnalogInput.class, "analogInput");

        // Wait for the game to start (driver press PLAY)
        waitForStart();

        // Run until the end of the match (driver press STOP)
        while (opModeIsActive()) {

            // Read the analog value from the sensor.
            double voltage = analogInput.getVoltage();
            servo.setPower(0.1);

            // Print the voltage to the driver station.
            telemetry.addData("Voltage:", voltage);
            telemetry.update();

            // You can also use the voltage to control other aspects of the robot, such as a servo.
            // Example:
            // servo.setPosition(voltage * 0.5 + 0.1); // A simple mapping of voltage to servo position

        }
    }
}

