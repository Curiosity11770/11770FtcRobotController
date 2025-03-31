package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "Meet0Auto", group = "Linear Opmode")
public class Meet0Auto extends LinearOpMode{
    Robot robot = new Robot(this);
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init();
        runtime.reset();

        waitForStart();

        runtime.reset();

        while(runtime.seconds() < 2){
            robot.drivetrain.leftFrontDrive.setPower(0.7);
            robot.drivetrain.rightFrontDrive.setPower(0.7);
            robot.drivetrain.leftBackDrive.setPower(0.7);
            robot.drivetrain.rightBackDrive.setPower(0.7);
        }

        robot.lift.liftToPositionPIDClass(700);
        robot.scoring.clawServo.setPosition(robot.scoring.CLAW_OPEN);
        robot.lift.liftToPositionPIDClass(0);
        runtime.reset();
        while(runtime.seconds() < 1){
            robot.drivetrain.leftFrontDrive.setPower(0.2);
            robot.drivetrain.rightFrontDrive.setPower(0.2);
            robot.drivetrain.leftBackDrive.setPower(0.2);
            robot.drivetrain.rightBackDrive.setPower(0.2);
        }
    }

}
