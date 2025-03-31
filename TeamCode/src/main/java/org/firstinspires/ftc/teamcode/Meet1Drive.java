package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "Meet1Drive", group = "Linear Opmode")
public class Meet1Drive extends LinearOpMode{
    Robot robot = new Robot(this);
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init();
        runtime.reset();

        waitForStart();

        runtime.reset();

        while(runtime.seconds() < 0.75){
            robot.drivetrain.leftFrontDrive.setPower(0.7);
            robot.drivetrain.rightFrontDrive.setPower(-0.7);
            robot.drivetrain.leftBackDrive.setPower(-0.7);
            robot.drivetrain.rightBackDrive.setPower(0.7);
        }
    }

}

