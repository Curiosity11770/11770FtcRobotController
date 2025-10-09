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

@Autonomous(name="PedroSpecimenAuto", group="Linear OpMode")
@Config
public class PedroSpecimenAuto extends LinearOpMode {

    private Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
    private Follower follower;
    public ElapsedTime timer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        pathTimer = new Timer();

        Pose startPose = new Pose(9, 57, 0);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        PathChain scorePreLoad = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.000, 57.000),
                                new Pose(23.063, 73.260),
                                new Pose(38.956, 74.22)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain retrieveGroundSamples = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Pose(38.956, 74.229),
                                new Pose(0.000, 4.070),
                                new Pose(105.626, 55.817),
                                new Pose(65.895, 20.738),
                                new Pose(22.481, 27.133)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Pose(22.481, 27.133),
                                new Pose(109.890, 19.187),
                                new Pose(22.481, 16.280)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Pose(22.481, 16.280),
                                new Pose(111.634, 10.078),
                                new Pose(22.481, 10.721)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain pickUpSpecimen = follower.pathBuilder()
                .addPath(   // Line 5
                        new BezierCurve(
                                new Pose(23.063, 10.272),
                                new Pose(23.451, 23.838),
                                new Pose(11.435, 23.838)
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
                new ParallelAction(
                        //lift.liftAction(Lift.EXT_HIGH_CHAMBER)
                ),
                //pedroBezierDriveToPose(new Pose(35, 70-cycle*2, 0), new Pose(16,68, 0)),
                pedroDriveOnPathChain(scorePreLoad)
                //add action to open claw
        ));

        //*******RETRIEVE GROUND SAMPLES********
        Actions.runBlocking(
                new ParallelAction(
                        //lift.liftAction(Lift.EXT_HIGH_CHAMBER)
                        //pedroBezierDriveToPose(new Pose(35, 70-cycle*2, 0), new Pose(16,68, 0)),
                        pedroDriveOnPathChain(retrieveGroundSamples)
                        //add action to open claw
                ));

        //*******RETRIEVE SPECIMEN********
        Actions.runBlocking(
                new SequentialAction(
                        pedroDriveOnPathChain(pickUpSpecimen)
                )
        );
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

    /*private Action buildDriveToSubmersible(MecanumDrive drive, double cycle) {
        int chamberOffset = 2;
        return drive.actionBuilder(initialPose)
                .waitSeconds(.4)
                .setTangent(Math.toRadians(90)) // Face upward initially
                .splineToLinearHeading(new Pose2d(24, 10 + cycle*chamberOffset, Math.toRadians(0)), Math.toRadians(0)) // Arrive facing forward
                .build();
    }

    private Action buildBackToObservationZone(MecanumDrive drive, Pose2d startingPose) {
        return drive.actionBuilder(startingPose)
                .strafeTo(new Vector2d(8, -24)) // Re-approach same point (demo)
                .build();
    }

    private Action buildCloseOut(MecanumDrive drive) {
        return drive.actionBuilder(new Pose2d(24, 10, Math.toRadians(0)))
                .strafeTo(new Vector2d(0, 0)) // Return to origin
                .build();
    }*/
}

