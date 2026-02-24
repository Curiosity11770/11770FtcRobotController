package org.firstinspires.ftc.teamcode.OpModes;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

@Autonomous(name="FarSideRedAuto", group="Linear OpMode")
@Config
public class FarSideRedAuto extends LinearOpMode {
    private Follower follower;
    private Paths myPaths;
    private Shooter shooter = new Shooter(this);
    private Intake intake = new Intake(this);
    private Spindexer spindexer = new Spindexer(this, intake, shooter);

    @Override

    public void runOpMode() throws InterruptedException {

        Pose startPose = new Pose(80, 10, Math.toRadians(90));
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
                pedroDriveOnPathChain(myPaths.SHOOT1, 1, true, 1),
                shooter.shooterAction(5900/60*shooter.TICKS_PER_REVOLUTION, 0.1))
        );

        Actions.runBlocking(new SequentialAction(shooter.transferAction(1,0.1),
                shooter.linkageAction(shooter.LINKAGE_UP, 0.1))
        );

        Actions.runBlocking(new SleepAction(3.0));

        Actions.runBlocking(spindexer.spindexerAction(0.75, 3.5));

        Actions.runBlocking(new SequentialAction(
                intake.intakeOn(0.1),
                new ParallelAction(
                        shooter.linkageAction(shooter.LINKAGE_DOWN, 0.1),
                        pedroDriveOnPathChain(myPaths.INTAKE1, 0.5, true, 4),
                        spindexer.autoIntake()
                )
        ));


        Actions.runBlocking(new SequentialAction(
                pedroDriveOnPathChain(myPaths.SHOOT2, 1, true, 1),
                        shooter.linkageAction(shooter.LINKAGE_UP, 0.1),
                        spindexer.spindexerAction(0.75, 3.5))
        );


        Actions.runBlocking(pedroDriveOnPathChain(myPaths.LEAVE, 1, true, 1));

    }

    private Action pedroDriveOnPathChain(PathChain targetPathChain, double maxPower, boolean holdPos, double seconds) {
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
                }else if (pathTimer.seconds() > seconds){
                    return false;
                }else{
                    return false;
                }
            }
        };
    }


    public static class Paths {
        public PathChain SHOOT1;
        public PathChain INTAKE1;
        public PathChain SHOOT2;
        public PathChain LEAVE;

        public Paths(Follower follower) {
            SHOOT1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(80, 10),

                                    new Pose(81, 20)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))

                    .build();

            INTAKE1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(81, 20),

                                    new Pose(137.104, 12.777)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))

                    .build();

            SHOOT2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(137.104, 12.777),

                                    new Pose(81, 20)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))

                    .build();

            LEAVE = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(81, 20),

                                    new Pose(98.490, 25.456)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }


}
