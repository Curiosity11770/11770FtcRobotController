package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "SubmersibleTest", group = "Linear Opmode")
public class SubmersibleTest extends LinearOpMode{
    Robot robot = new Robot(this);
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init();
        runtime.reset();

        waitForStart();
        robot.timer.reset();

        runtime.reset();

        while (opModeIsActive() && !isStopRequested()){
            if (runtime.seconds() < 3.0) {
                robot.driveToHuskyLens();
            }
            if (runtime.seconds() > 3.0 && runtime.seconds() < 6.0) {
                robot.submersibleIntake();
            }
            telemetry.update();
        }
    }

}

