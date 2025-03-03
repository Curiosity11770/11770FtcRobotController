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

        runtime.reset();
        robot.driveToHuskyLens();

        while(runtime.seconds() < 2){
            robot.driveToHuskyLens();
        }
    }

}

