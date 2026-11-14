package org.firstinspires.ftc.teamcode.autonomous;

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
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.RobotHardware;


@Autonomous(name = "SecondSideShoot", group = "Autonomous")
public class SecondSideShoot extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(9.0074, 132.6871, 0);
    private final Pose path1 = poseFactory.of(59.1328, 132.2827, 90);
    private final Pose path1Control1 = poseFactory.of(58.32, 100.5059, 0);
    private final Pose point2Start = poseFactory.of(59.1328, 132.2827, 90);
    private final Pose point2 = poseFactory.of(11.5598, 116.7052, 0);
    private final Pose point2Control1 = poseFactory.of(43.781, 112.1581, 0);

    RobotHardware robot;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> robot.transfer.on()),
                instant(() -> robot.launcher.launcherOn()),
                follow(follower, path1()),
                // needs to wait longer
                race(waitUntil(() ->
                                robot.launcher.atVelocity(robot.launcher.pollenLauncher, robot.launcher.POLLEN_RPM)),
                        waitMs(6000)
                ),
                instant(() -> robot.transfer.openGate()),
                waitMs(1000),
                instant(() -> robot.transfer.closeGate()),
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
        robot.init();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
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
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }
}
