package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

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
import com.pedropathing.geometry.BezierPoint;
;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.geometry.BezierLine;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="DrivingTest", group="Linear OpMode")
@Config
public class DrivingTest extends LinearOpMode {

    private Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
    private Follower follower;
    public ElapsedTime timer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        pathTimer = new Timer();

        Pose startPose = new Pose(0, 0, 0);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        PathChain forwards = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(0, 0),
                                new Pose(48, 0)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain backwards = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Pose(48, 0),
                                new Pose(0.0, 0)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        waitForStart();

        int cycle = 0;

        if (isStopRequested()) return;
        //*******SCORE PRELOAD********
        //Score preload on submersible
        Actions.runBlocking(new SequentialAction(
                //pedroBezierDriveToPose(new Pose(35, 70-cycle*2, 0), new Pose(16,68, 0)),
                pedroDriveOnPathChain(forwards)
                //add action to open claw
        ));

        //*******RETRIEVE GROUND SAMPLES********
        Actions.runBlocking(
                new ParallelAction(
                        //lift.liftAction(Lift.EXT_HIGH_CHAMBER)
                        //pedroBezierDriveToPose(new Pose(35, 70-cycle*2, 0), new Pose(16,68, 0)),
                        pedroDriveOnPathChain(backwards)
                        //add action to open claw
                ));

        //*******RETRIEVE SPECIMEN********
        //Actions.runBlocking(pedroDriveToPose(new Pose(24, 0, 0)));
        /*
        while(opModeIsActive() && cycle < 5) {
            // Run individual trajectories
            Actions.runBlocking(new ParallelAction(
                    //buildDriveToSubmersible(drive, cycle),
                    pedroDriveToPose(new Pose(24, 10+cycle*2, 0)),
                    new SequentialAction(
                            scoring.pivotAction(Scoring.PIVOT_ZERO),
                            lift.liftAction(Lift.EXT_HIGH_CHAMBER)
                    ),
                    scoring.clawSubmersible())
            );
            Actions.runBlocking(new ParallelAction(
                    pedroDriveToPose(new Pose(0, -24, 0)),
                    lift.liftAction(Lift.EXT_RETRACTED),
                    scoring.pivotAction(Scoring.PIVOT_OBSERVATION_ZONE),
                    scoring.clawToObservationZone())
            );
            cycle++;
        }

         */


        // Run sequence of two new, fresh actions
        /*
        Actions.runBlocking(
                new SequentialAction(
                        buildBackToSub(drive),
                        buildCloseOut(drive)
                )
        );

         */

        /*
        // Additional scoring/lift logic can go here
        Actions.runBlocking(scoring.clawToObservationZone());
        Actions.runBlocking(scoring.clawSubmersible());
        Actions.runBlocking(scoring.pivotAction(Scoring.PIVOT_OBSERVATION_ZONE));
        Actions.runBlocking(scoring.pivotAction(Scoring.PIVOT_ZERO));
        Actions.runBlocking(lift.liftAction(Lift.EXT_LOW_BASKET));
        Actions.runBlocking(lift.liftAction(Lift.EXT_RETRACTED));

        Actions.runBlocking(new ParallelAction(
                scoring.pivotObservationZone(),
                scoring.clawToObservationZone()
        ));
        Actions.runBlocking(new ParallelAction(
                scoring.pivotSubmersible(),
                scoring.clawSubmersible()
        ));
        */

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

    private Action pedroDriveToPose(Pose targetPose){
        return new Action() {
            private boolean initialized = false;
            Path newPath = null;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    timer.reset();
                    Pose initialPose = follower.getPose();
                    newPath = new Path(new BezierLine(initialPose, targetPose));
                    newPath.setLinearHeadingInterpolation(initialPose.getHeading(),targetPose.getHeading());
                    follower.followPath(newPath);
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

    private Action pedroDriveOnPath(Path targetPath){
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    timer.reset();
                    follower.followPath(targetPath);
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

}

