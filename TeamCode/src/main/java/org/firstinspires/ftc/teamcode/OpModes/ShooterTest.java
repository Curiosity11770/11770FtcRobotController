package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ShooterTest", group = "Linear OpMode")


public class ShooterTest extends LinearOpMode {
    Shooter shooter = new Shooter(this);
    private ElapsedTime runtime = new ElapsedTime();

    private double velocity = 500; // starting velocity (ticks per second)
    private final double VELOCITY_INCREMENT = 100; // how much to change per button press
    private final double MAX_VELOCITY = 7000; // limit to avoid over-speeding
    private final double MIN_VELOCITY = 0;

    private boolean aPressedLast = false;
    private boolean bPressedLast = false;

    public void runOpMode() {
        runtime.reset();
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a && !aPressedLast) {
                velocity += VELOCITY_INCREMENT;
                if (velocity > MAX_VELOCITY) velocity = MAX_VELOCITY;
            }

            // Button B: Decrease velocity
            if (gamepad1.b && !bPressedLast) {
                velocity -= VELOCITY_INCREMENT;
                if (velocity < MIN_VELOCITY) velocity = MIN_VELOCITY;
            }

            // Update flags
            aPressedLast = gamepad1.a;
            bPressedLast = gamepad1.b;

            // Set the motor velocity
            shooter.shootingMotor.setVelocity(velocity);


            telemetry.update();
            telemetry.addData("rpm", velocity);
        }
    }

}
