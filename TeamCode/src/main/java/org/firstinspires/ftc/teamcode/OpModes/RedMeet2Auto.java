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
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@Autonomous(name="RedMeet2Auto", group="Linear OpMode")
@Config
public class RedMeet2Auto extends LinearOpMode {
    private Follower follower;
    private Paths myPaths;
    private Shooter shooter = new Shooter(this);
    private Spindexer spindexer = new Spindexer(this);
    private Intake intake = new Intake(this);

    @Override

    public void runOpMode() throws InterruptedException {

        Pose startPose = new Pose(111, 136, Math.toRadians(90));
        follower =
                Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        shooter.init();
        spindexer.init();
        intake.init();

        myPaths = new Paths(follower);

        waitForStart();

        if (isStopRequested()) return;
        spindexer.colorUpdate(spindexer.hsvValuesOne, spindexer.rgb0);
        spindexer.colorUpdate(spindexer.hsvValuesTwo, spindexer.rgb1);
        spindexer.colorUpdate(spindexer.hsvValuesThree,spindexer.rgb2);

        //Drive back to scan obelisk
        Actions.runBlocking(new ParallelAction(
                pedroDriveOnPathChain(myPaths.DRIVEBACKTOLOOK, 1, true),
                shooter.shooterAction(shooter.REVOLUTIONS_PER_MINUTE/60*shooter.TICKS_PER_REVOLUTION, 0.1),
                shooter.transferAction(0.7, 0.1)
        ));

        //Align with goal and launch artifacts
        Actions.runBlocking(new SequentialAction(
                pedroDriveOnPathChain(myPaths.SHOOTPATH1, 0.7, true),
                new ParallelAction(
                        shooter.linkageAction(shooter.LINKAGE_UP, 0.1),
                        spindexer.spindexerAction(0.1, 3.5))
        ));

        Actions.runBlocking(pedroDriveOnPathChain(myPaths.FACEBALL1, 0.7, true));

        Actions.runBlocking(new SequentialAction(
                intake.intakeOn(0.2),
                shooter.linkageOff(0.1),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.DRIVEINTOBALLS1, 0.4, true),
                        spindexer.autoIntake()
                )
        ));

        Actions.runBlocking(new SequentialAction(
                pedroDriveOnPathChain(myPaths.SHOOTPATH2, 0.7, true),
                shooter.transferAction(.7,0.1),
                new ParallelAction(
                        shooter.linkageAction(shooter.LINKAGE_UP, 0.1),
                        spindexer.spindexerAction(0.1, 3.5))
        ));

        Actions.runBlocking(pedroDriveOnPathChain(myPaths.FACEBALL2, 0.7, true));

        Actions.runBlocking(new SequentialAction(
                intake.intakeOn(0.2),
                shooter.linkageOff(0.1),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.DRIVEINTOBALLS2, 0.3, true),
                        spindexer.autoIntake()
                )
        ));

        Actions.runBlocking(new SequentialAction(
                pedroDriveOnPathChain(myPaths.SHOOTPATH3, 0.7, true),
                shooter.transferAction(.7,0.1),
                new ParallelAction(
                        shooter.linkageAction(shooter.LINKAGE_UP, 0.1),
                        spindexer.spindexerAction(0.1, 3.5))
        ));


        Actions.runBlocking(pedroDriveOnPathChain(myPaths.STRAFEPATH, 1, true));

    }

    private Action pedroDriveOnPathChain(PathChain targetPathChain, double maxPower, boolean holdPos) {
        return new Action() {
            private boolean initialized = false;
            ElapsedTime pathTimer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    pathTimer.reset();
                    follower.followPath(targetPathChain, maxPower,holdPos);
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

        public PathChain DRIVEBACKTOLOOK;
        public PathChain SHOOTPATH1;
        public PathChain FACEBALL1;
        public PathChain DRIVEINTOBALLS1;
        public PathChain SHOOTPATH2;
        public PathChain FACEBALL2;
        public PathChain DRIVEINTOBALLS2;
        public PathChain SHOOTPATH3;
        public PathChain STRAFEPATH;

        public Paths(Follower follower) {

            DRIVEBACKTOLOOK = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(111.1436, 135.590), new Pose(91.1436, 100.749))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            SHOOTPATH1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(91.1436, 100.749), new Pose(95, 97))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(46))
                    .build();

            FACEBALL1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95, 95.25), new Pose(95, 85.928))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(0))
                    .build();

            DRIVEINTOBALLS1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95, 85.928), new Pose(126.572, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            SHOOTPATH2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(126.572, 84.000), new Pose(94, 93.712))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(46))
                    .build();

            FACEBALL2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(94, 93.712), new Pose(94, 60.072))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(0))
                    .build();

            DRIVEINTOBALLS2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(94, 60.072), new Pose(131, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            SHOOTPATH3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(129, 60.000), new Pose(92.025, 92.510))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(46))
                    .build();

            STRAFEPATH = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(92.025, 92.510), new Pose(107, 82.510))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(90))
                    .build();
        }
    }


}
