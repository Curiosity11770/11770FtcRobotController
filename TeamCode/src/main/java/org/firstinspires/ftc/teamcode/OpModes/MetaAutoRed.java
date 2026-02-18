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

@Autonomous(name="MetaAutoRed", group="Linear OpMode")
@Config
public class MetaAutoRed extends LinearOpMode {
    private Follower follower;
    private Paths myPaths;
    private Shooter shooter = new Shooter(this);
    private Intake intake = new Intake(this);
    private Spindexer spindexer = new Spindexer(this, intake);

    @Override

    public void runOpMode() throws InterruptedException {

        Pose startPose = new Pose(111.16, 136.58, Math.toRadians(90));
        follower = Constants.createFollower(hardwareMap);
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


        //Align with goal and launch artifacts
        Actions.runBlocking(new ParallelAction(
                pedroDriveOnPathChain(myPaths.TURNANDSHOOT, 0.7, true),
                shooter.shooterAction(shooter.REVOLUTIONS_PER_MINUTE/60*shooter.TICKS_PER_REVOLUTION, 0.1),
                shooter.transferAction(.7,0.1),
                new ParallelAction(
                        shooter.linkageAction(shooter.LINKAGE_UP, 0.1),
                        spindexer.spindexerAction(0.1, 3.5))
        ));

        Actions.runBlocking(pedroDriveOnPathChain(myPaths.DRIVETOBALLS1, 0.7, true));

        Actions.runBlocking(new SequentialAction(
                intake.intakeOn(0.5),
                shooter.linkageOff(0.1),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.DRIVEINTOBALLS1, 0.3, true),
                        spindexer.autoIntake()
                )
        ));

        Actions.runBlocking(pedroDriveOnPathChain(myPaths.CLEARCLASSIFIER1, 0.7, true));
        Actions.runBlocking(pedroDriveOnPathChain(myPaths.HOLDCLASSIFIER1, 0.7, true));

        Actions.runBlocking(new SequentialAction(
                pedroDriveOnPathChain(myPaths.SHOOTPATH2, 0.7, true),
                shooter.transferAction(.7,0.1),
                new ParallelAction(
                        shooter.linkageAction(shooter.LINKAGE_UP, 0.1),
                        spindexer.spindexerAction(0.1, 3.5))
        ));

        Actions.runBlocking(pedroDriveOnPathChain(myPaths.INTAKEPATH2, 0.7, true));

        Actions.runBlocking(new SequentialAction(
                intake.intakeOn(0.5),
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


        Actions.runBlocking(pedroDriveOnPathChain(myPaths.STRAFEOUT, 1, true));

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

        public PathChain TURNANDSHOOT;
        public PathChain DRIVETOBALLS1;
        public PathChain DRIVEINTOBALLS1;
        public PathChain CLEARCLASSIFIER1;
        public PathChain HOLDCLASSIFIER1;
        public PathChain SHOOTPATH2;
        public PathChain INTAKEPATH2;
        public PathChain DRIVEINTOBALLS2;
        public PathChain SHOOTPATH3;
        public PathChain STRAFEOUT;

        public Paths(Follower follower) {
            TURNANDSHOOT = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(111.166, 136.586), new Pose(111.000, 110.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(50))
                    .build();

            DRIVETOBALLS1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(111.000, 110.000), new Pose(108.566, 83.406))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                    .build();

            DRIVEINTOBALLS1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(108.566, 83.406), new Pose(130.000, 83.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            CLEARCLASSIFIER1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(130.000, 83.000), new Pose(130.000, 66.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            HOLDCLASSIFIER1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(130.000, 66.000), new Pose(132.000, 66.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            SHOOTPATH2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(132.000, 66.000), new Pose(111.000, 110.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();

            INTAKEPATH2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(111.000, 110.000), new Pose(106.538, 59.406))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))
                    .build();

            DRIVEINTOBALLS2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(106.538, 59.406), new Pose(131.521, 59.611))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            SHOOTPATH3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(131.521, 59.611), new Pose(111.144, 109.927))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))
                    .build();


            STRAFEOUT = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(111.144, 109.927), new Pose(111.837, 90.186))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(50))
                    .build();
        }
    }
}
