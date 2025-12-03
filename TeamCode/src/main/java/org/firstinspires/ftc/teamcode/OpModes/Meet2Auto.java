package org.firstinspires.ftc.teamcode.OpModes;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@Autonomous(name="Meet2Auto", group="Linear OpMode")
@Config
public class Meet2Auto extends LinearOpMode {
    private Follower follower;
    private Paths myPaths;
    private Shooter shooter = new Shooter(this);
    private Spindexer spindexer = new Spindexer(this);

    @Override

    public void runOpMode() throws InterruptedException {

        Pose startPose = new Pose(32.35, 136, Math.toRadians(90));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        shooter.init();
        spindexer.init();

        myPaths = new Paths(follower);

        waitForStart();

        if (isStopRequested()) return;
        /*
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                    pedroDriveOnPathChain(myPaths.Drivebacktolook)/*,
                    shooter.shooterAction(shooter.REVOLUTIONS_PER_MINUTE/60*shooter.TICKS_PER_REVOLUTION, 0.1),
                    shooter.transferAction(0.7, 0.1)),
                pedroDriveOnPathChain(myPaths.ShootPath1),
                new ParallelAction(
                    shooter.linkageAction(shooter.LINKAGE_UP, 0.1),
                    spindexer.spindexerAction(0.1, 7))
        ));

         */
        Actions.runBlocking(pedroDriveOnPathChain(myPaths.Drivebacktolook));
        Actions.runBlocking(pedroDriveOnPathChain(myPaths.ShootPath1));

    }

    private Action pedroDriveOnPathChain(PathChain targetPathChain) {
        return new Action() {
            private boolean initialized = false;
            ElapsedTime pathTimer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    pathTimer.reset();
                    follower.followPath(targetPathChain, true);
                }


                follower.update();

                telemetry.addData("pathtimer", pathTimer.seconds());
                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());
                telemetry.addData("isbusy", follower.isBusy());
                telemetry.addData("distance remaining",follower.getDistanceRemaining());
                telemetry.addData("At pose",follower.atPose(targetPathChain.endPose(),2,2,Math.toRadians(10)));
                telemetry.addData("T-Value",follower.getCurrentTValue());
                telemetry.addData("path completion",follower.getPathCompletion());
                telemetry.addData("following pathchain",follower.getFollowingPathChain());
                telemetry.addData("parametric end",follower.atParametricEnd());

                //telemetry.addData("path", targetPathChain);

                telemetry.update();

                if(follower.isBusy()){
                    return true;
                }else{
                    return false;
                }
            }
        };
    }

    public static class Paths {

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
