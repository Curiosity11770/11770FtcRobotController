package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Linear Opmode")

public class TeleOp extends LinearOpMode {
    Robot robot = new Robot(this);
    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode() {
        robot.init();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            robot.teleOp();

            telemetry.update();
        }
    }

}
