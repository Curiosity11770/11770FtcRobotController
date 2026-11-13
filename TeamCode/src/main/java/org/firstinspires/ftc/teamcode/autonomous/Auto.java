package org.firstinspires.ftc.teamcode.autonomous;

import static com.pedropathing.api.Paths.*;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.RobotHardware;

@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends LinearOpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose path1 = poseFactory.of(9.4251, 8.0133, 180);
    private final Pose path1Control1 = poseFactory.of(49.5983, 21.5888, 0);
    private final Pose point2Start = poseFactory.of(9.4251, 8.0133, 180);
    private final Pose point2 = poseFactory.of(60.299, 133.1236, 90);
    private final Pose point2Control1 = poseFactory.of(38.8005, 114.5127, 0);
    private final Pose point2Control2 = poseFactory.of(7.6102, 106.9326, 0);
    private final Pose point2Control3 = poseFactory.of(61.8781, 103.8863, 0);

    RobotHardware robot;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                parallel(follow(follower, path1()), robot.launcher.launcherOn(3700)),  // spin up while driving
                waitMs(500),
                robot.launcher.launcherOff(),
                follow(follower, path2())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        robot = new RobotHardware(this);

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {

            Actions.runBlocking(new SequentialAction(

            ));

            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2Control2, point2Control3, point2).linear(point2Start, point2);
    }
}