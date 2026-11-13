package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "SecondSideShoot", group = "Autonomous")
public class SecondSideShoot extends LinearOpMode {
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(8.6901, 133.1367, 0);
    private final Pose leavetoshoot = poseFactory.of(59.2029, 130.3678, 90);
    private final Pose leavetoshootControl1 = poseFactory.of(55.4434, 108.245, 0);
    private final Pose leaveshoot1Start = poseFactory.of(59.2029, 130.3678, 90);
    private final Pose leaveshoot1 = poseFactory.of(46.7496, 117.8832, 90);
    private final Pose leaveshoot1Control1 = poseFactory.of(64.3745, 115.9974, 0);
    private final Pose intakeflowerStart = poseFactory.of(46.7496, 117.8832, 90);
    private final Pose intakeflower = poseFactory.of(46.7442, 127.1787, 90);
    private final Pose shoot2Start = poseFactory.of(46.7442, 127.1787, 90);
    private final Pose shoot2 = poseFactory.of(60.0774, 132.5243, 90);
    private final Pose shoot2Control1 = poseFactory.of(53.4645, 124.4425, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
            follow(follower, leavetoshoot()),
            follow(follower, leaveshoot1()),
            follow(follower, intakeflower()),
            follow(follower, shoot2())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

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

    public Path leavetoshoot() {
        return Paths.curve(start, leavetoshootControl1, leavetoshoot).linear(start, leavetoshoot);
    }

    public Path leaveshoot1() {
        return Paths.curve(leaveshoot1Start, leaveshoot1Control1, leaveshoot1).linear(leaveshoot1Start, leaveshoot1);
    }

    public Path intakeflower() {
        return Paths.line(intakeflowerStart, intakeflower).linear(intakeflowerStart, intakeflower);
    }

    public Path shoot2() {
        return Paths.curve(shoot2Start, shoot2Control1, shoot2).linear(shoot2Start, shoot2);
    }
}
