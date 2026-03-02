package org.firstinspires.ftc.teamcode.OpModes;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.ParallelOTOSEncoder;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Autonomous(name="RedTwelveBallClassfied", group="Linear OpMode")
@Config
public class RedTwelveBallClassfied extends LinearOpMode {
    private Follower follower;
    private Paths myPaths;
    private Shooter shooter = new Shooter(this);
    private Vision vision = new Vision(this);
    private Intake intake = new Intake(this);
    private Spindexer spindexer = new Spindexer(this, intake, shooter);

    int id = 21;

    @Override

    public void runOpMode() throws InterruptedException {

        Pose startPose = new Pose(111.75, 136, Math.toRadians(90));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        shooter.init();
        vision.init();
        spindexer.init();
        intake.init();

        myPaths = new Paths(follower);

        spindexer.COLOR_STATUS = new Spindexer.ColorMode[]{Spindexer.ColorMode.PURPLE, Spindexer.ColorMode.GREEN, Spindexer.ColorMode.PURPLE};
        waitForStart();

        if (isStopRequested()) return;


        //Drive back to scan obelisk
        Actions.runBlocking(new ParallelAction(spindexer.setColorStatus(Spindexer.ColorMode.PURPLE, Spindexer.ColorMode.GREEN, Spindexer.ColorMode.PURPLE),
                pedroDriveOnPathChain(myPaths.DRIVEBACKTOLOOK, 1, true),
                shooter.shooterAction(3300/60*shooter.TICKS_PER_REVOLUTION, 0.05),
                shooter.transferAction(shooter.TRANSFER_SPEED, 0.05)
        ));

        Actions.runBlocking(scanAprilTags());

        //Align with goal and launch artifacts
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        spindexer.setMotifOrder(id), spindexer.shootingMotif(id),
                    pedroDriveOnPathChain(myPaths.SHOOTPATH1, 1, true)),
                shooter.linkageAction(shooter.LINKAGE_UP, 0.1),
                spindexer.spindexerAction(0.75, 1.25),
                pedroDriveOnPathChain(myPaths.FACEBALL2, 1, true),
                intake.intakeOn(0.1),
                shooter.linkageAction(shooter.LINKAGE_DOWN, 0.01),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.DRIVEINTOBALLS2, 0.3, true),
                        spindexer.autoIntake()
                ),
                pedroDriveOnPathChain(myPaths.CLEARCLASSFIER, 1, true),
                pedroDriveOnPathChain(myPaths.CLEARCLASSFIER2, 1, true),
                pedroDriveOnPathChain(myPaths.CLEARCLASSFIER3, 1, true),
                spindexer.setColorStatus(Spindexer.ColorMode.PURPLE, Spindexer.ColorMode.PURPLE, Spindexer.ColorMode.GREEN),
                spindexer.shootingMotif2(id),
                pedroDriveOnPathChain(myPaths.SHOOTPATH3, 1, true),
                shooter.linkageAction(shooter.LINKAGE_UP, 0.1),
                spindexer.spindexerAction(0.75, 1.25),
                new ParallelAction(
                        spindexer.setColorStatus(Spindexer.ColorMode.PURPLE, Spindexer.ColorMode.PURPLE, Spindexer.ColorMode.GREEN),
                        spindexer.shootingMotif2(id),
                        pedroDriveOnPathChain(myPaths.SHOOTPATH3, 1, true)),
                shooter.linkageAction(shooter.LINKAGE_UP, 0.1),
                spindexer.spindexerAction(0.75, 1.25),
                pedroDriveOnPathChain(myPaths.FACEBALL1, 1, true),
                intake.intakeOn(0.01),
                shooter.linkageAction(shooter.LINKAGE_DOWN, 0.01),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.DRIVEINTOBALLS1, 0.3, true),
                        spindexer.autoIntake()
                ),
                new ParallelAction(
                    spindexer.setColorStatus(Spindexer.ColorMode.PURPLE, Spindexer.ColorMode.GREEN, Spindexer.ColorMode.PURPLE),
                    spindexer.shootingMotif(id),
                    pedroDriveOnPathChain(myPaths.SHOOTPATH2, 1, true)),
                shooter.linkageAction(shooter.LINKAGE_UP, 0.1),
                spindexer.spindexerAction(0.75, 1.25),
                pedroDriveOnPathChain(myPaths.FACEBALL3, 1, true),
                intake.intakeOn(0.1),
                shooter.linkageAction(shooter.LINKAGE_DOWN, 0.01),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.DRIVEINTOBALLS3, 0.3, true),
                        spindexer.autoIntake()
                ),
                new ParallelAction(spindexer.setColorStatus(Spindexer.ColorMode.PURPLE, Spindexer.ColorMode.GREEN, Spindexer.ColorMode.PURPLE),
                        spindexer.shootingMotif3(id),
                        pedroDriveOnPathChain(myPaths.SHOOTPATH4, 1, true)),
                shooter.linkageAction(shooter.LINKAGE_UP, 0.1),
                spindexer.spindexerAction(0.75, 1.25),
                pedroDriveOnPathChain(myPaths.STRAFEPATH, 1, true)

        ));

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
                //telemetry.addData("distance remaining",follower.getDistanceRemaining());
                telemetry.addData("At pose",follower.atPose(targetPathChain.endPose(),2,2,Math.toRadians(10)));
                //telemetry.addData("T-Value",follower.getCurrentTValue());
                telemetry.addData("path completion",follower.getPathCompletion());
                //telemetry.addData("following pathchain",follower.getFollowingPathChain());
                //telemetry.addData("parametric end",follower.atParametricEnd());

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

    private Action scanAprilTags() {
        return new Action() {
            private boolean initialized = false;
            ElapsedTime tagTimer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    tagTimer.reset();
                }

                vision.result = vision.limelight.getLatestResult();
                vision.fiducials = vision.result.getFiducialResults();

                for (LLResultTypes.FiducialResult fiducial : vision.fiducials) {
                    id = fiducial.getFiducialId();
                }
                telemetry.addData("id",id);

                telemetry.update();

                if(tagTimer.seconds() < 0.1){
                    return true;
                } else {
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
        public PathChain FACEBALL3;
        public PathChain DRIVEINTOBALLS3;
        public PathChain SHOOTPATH4;
        public PathChain CLEARCLASSFIER;
        public PathChain CLEARCLASSFIER2;
        public PathChain CLEARCLASSFIER3;

        public PathChain STRAFEPATH;

        public Paths(Follower follower) {

            DRIVEBACKTOLOOK = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(111.905, 135.590), new Pose(91.137, 100.749))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                    .build();

            SHOOTPATH1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(91.137, 100.749), new Pose(95.6, 97))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(47))
                    .build();

            FACEBALL2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.6, 97), new Pose(102.398, 64.072))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(47), Math.toRadians(0))
                    .build();

            DRIVEINTOBALLS2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.398, 64.072), new Pose(128.5, 64.072))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            CLEARCLASSFIER = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(128.5, 64.072),
                                    new Pose(125.5, 69.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            CLEARCLASSFIER2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.5, 69.500),
                                    new Pose(128.5, 69.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            CLEARCLASSFIER3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(128.5, 69.500),
                                    new Pose(108.500, 69.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            SHOOTPATH3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(108.500, 69.500), new Pose(93.025, 92.510))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(47))
                    .build();

            FACEBALL1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(93.025, 92.510), new Pose(102.428, 85.928))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(47), Math.toRadians(0))
                    .build();

            DRIVEINTOBALLS1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.428, 85.928), new Pose(120, 85.900))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            SHOOTPATH2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120, 85.900), new Pose(102.226, 93.712))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(47))
                    .build();

            FACEBALL3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.226, 93.712), new Pose(102.226, 38.572))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(47), Math.toRadians(0))
                    .build();

            DRIVEINTOBALLS3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.226, 38.572), new Pose(125.5, 38.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            SHOOTPATH4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.5, 38.500), new Pose(93.025, 92.510))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))
                    .build();

            STRAFEPATH = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(93.025, 92.510), new Pose(108.025, 82.510))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(90))
                    .build();
        }
    }


}
