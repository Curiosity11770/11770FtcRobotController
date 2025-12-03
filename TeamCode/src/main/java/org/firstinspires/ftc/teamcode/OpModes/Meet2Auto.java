package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.geometry.BezierLine;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Meet2Auto", group="Linear OpMode")
@Config
public class Meet2Auto extends LinearOpMode {
    private Follower follower;
    public ElapsedTime timer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer;

    @Override

    public void runOpMode() throws InterruptedException{
        pathTimer = new Timer();

        Pose startPose = new Pose(56, 136, 90);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        Paths myPaths = new Paths(follower);

        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(new ParallelAction(
                pedroDriveOnPathChain(myPaths.Drivebacktolook)
                )
        );


    }

    private Action pedroDriveOnPathChain(PathChain targetPathChain) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    timer.reset();
                    follower.followPath(targetPathChain);
                }

                follower.update();


                telemetry.addData("timer", timer.seconds());
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                telemetry.update();

                return follower.isBusy();
            }
        };
    }
    public class Paths {

        public PathChain Drivebacktolook;
        public PathChain ShootPath1;
        public PathChain FaceBall1;
        public PathChain DRIVEINTOBALLS1;
        public PathChain SHOOTPATH2;
        public PathChain FaceBall2;
        public PathChain DRIVEINTOBALLS2;
        public PathChain SHOOTPATH3;

        public Paths(Follower follower) {
            Drivebacktolook = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(32.357, 135.602), new Pose(56.000, 89.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .setReversed()
                    .build();

            ShootPath1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 89.000), new Pose(56.000, 89.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(130))
                    .build();

            FaceBall1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 89.000), new Pose(56.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            DRIVEINTOBALLS1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 84.000), new Pose(14.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            SHOOTPATH2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(14.000, 84.000), new Pose(56.000, 89.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            FaceBall2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 89.000), new Pose(56.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            DRIVEINTOBALLS2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 60.000), new Pose(14.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            SHOOTPATH3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(14.000, 60.000), new Pose(56.000, 89.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();
        }
    }


}
