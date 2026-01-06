package org.firstinspires.ftc.teamcode.OpModes;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@Autonomous(name="LeaveAuto", group="Linear OpMode")
@Config
public class LeaveAuto extends LinearOpMode {
    public Robot robot = new Robot(this);
    public ElapsedTime timer = new ElapsedTime();

    @Override

    public void runOpMode() throws InterruptedException {

        robot.init();
        timer.reset();

        waitForStart();
        while (timer.seconds() < 1) {
            robot.drivetrain.leftBackDrive.setPower(-0.5);
            robot.drivetrain.leftFrontDrive.setPower(0.5);
            robot.drivetrain.rightBackDrive.setPower(0.5);
            robot.drivetrain.rightFrontDrive.setPower(-0.5);

        }
        while (timer.seconds() > 1.5) {
            robot.drivetrain.leftBackDrive.setPower(0);
            robot.drivetrain.leftFrontDrive.setPower(0);
            robot.drivetrain.rightBackDrive.setPower(0);
            robot.drivetrain.rightFrontDrive.setPower(0);
        }

    }

}
