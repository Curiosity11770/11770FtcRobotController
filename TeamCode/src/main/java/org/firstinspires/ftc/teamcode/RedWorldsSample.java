package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Scoring;

@Config
@Autonomous(name = "RedWorldsSample", group = "Linear OpMode")
public class RedWorldsSample extends LinearOpMode {

    Robot robot;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime totalTime = new ElapsedTime();
    Boolean strafe = false;


    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    //TODO Update states to reflect flow of robot actions
    enum State {
        DRIVE_TO_BASKET,
        DELIVER_SAMPLE,
        DRIVE_TO_FLOOR_SAMPLE,
        RETRIEVE_FLOOR_SAMPLE,
        TRANSFER,
        DRIVE_TO_SUBMERSIBLE,
        RETRIEVE_SUBMERSIBLE_SAMPLE,
        DRIVE_TO_BASKET_FROM_SUBMERSIBLE,
        PARK,
        IDLE
    }

    //Constants so these can be tuned in the dashboard
    public static double basketX = 12;
    public static double basketY = 12.3;
    public static double basketHeading = -45;

    public static double floorSampleX = 11;
    public static double floorSampleY = 20;
    public static double floorSampleHeading = -20;

    public static double submersibleX = 51.5;
    public static double submersibleY = -16;
    public static double submersibleHeading = -90;

    public static double parkingX = 48;
    public static double parkingY = -20;
    public static double parkingHeading = -90;

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.DRIVE_TO_BASKET;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    int samplesScored;


    @Override
    public void runOpMode() {
        //calling constructor
        robot = new Robot(this);

        //calling init function
        robot.init();

        samplesScored = 0;

        //TODO Pass starting pose to localizer
        //for Gobilda it looks like this
        //robot.drivetrain.localizer.odo.setPosition(startPose);
        //for sparkfun it looks like this
        robot.drivetrain.localizer.odo.setPosition(startPose);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        timer.reset();
        totalTime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case DRIVE_TO_BASKET:
                    robot.intake.spinIntake.setPower(0);
                    robot.drivetrain.profiledDriveToTarget(basketX, basketY, basketHeading);
                    robot.lift.liftMode = Lift.LiftMode.HIGH_BASKET;
                    robot.scoring.scoringMode = Scoring.ScoringMode.SAMPLE;
                    //put condition for switch at the end, condition can be based on time or completion of a task
                    if (timer.seconds() > 1.5) {
                        switchState(State.DELIVER_SAMPLE);
                    }
                    break;
                case DELIVER_SAMPLE:
                    robot.drivetrain.driveTime(-0.4, 0.5, timer);
                    if (timer.seconds() > 0.5) {
                        robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    }
                    if (timer.seconds() > 0.6 && samplesScored < 3) {
                        switchState(State.DRIVE_TO_FLOOR_SAMPLE);
                        samplesScored++;
                    } else if (timer.seconds() > 0.7) {
                        switchState(State.DRIVE_TO_SUBMERSIBLE);
                        samplesScored++;
                    }
                    break;
                case DRIVE_TO_FLOOR_SAMPLE:
                    robot.extension.leftLink.setPosition(0.5);
                    robot.extension.rightLink.setPosition(0.5);
                    robot.intake.intakeMode = Intake.IntakeMode.INTAKE;
                    robot.intake.spinIntake.setPower(1);
                    telemetry.addData("expression", floorSampleY + samplesScored * 6.0);
                    if (samplesScored == 1) {
                        robot.drivetrain.profiledDriveToTarget(floorSampleX, floorSampleY, floorSampleHeading + (samplesScored - 1) * 22.0);
                    } else if (samplesScored == 2) {
                        robot.drivetrain.profiledDriveToTarget(floorSampleX - 2, floorSampleY - 1, 0.4);
                    } else if (samplesScored == 3) {
                        robot.drivetrain.profiledDriveToTarget(floorSampleX + 4, 10.5, floorSampleHeading + (samplesScored - 1) * 30);

                    }
                    if (timer.seconds() > 1.5) {
                        switchState(State.RETRIEVE_FLOOR_SAMPLE);
                    }
                    break;
                case RETRIEVE_FLOOR_SAMPLE:
                    robot.lift.liftMode = Lift.LiftMode.GROUND;
                    robot.scoring.scoringMode = Scoring.ScoringMode.TRANSFER;
                    robot.drivetrain.relativeDriveToTarget(12, 0, 0, 0.03);
                    if (timer.seconds() > 1.5 || robot.intake.colors.green > 0.01) {
                        switchState(State.TRANSFER);
                    }
                    break;
                case TRANSFER:
                    robot.extension.leftLink.setPosition(0.87);
                    robot.extension.rightLink.setPosition(0.87);
                    robot.intake.spinIntake.setPower(1);
                    robot.intake.intakeMode = Intake.IntakeMode.TRANSFER;
                    robot.drivetrain.profiledDriveToTarget(basketX, basketY, basketHeading);
                    if (timer.seconds() > 1.0) {
                        robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                    }
                    if (timer.seconds() > 1.2) {
                        switchState(State.DRIVE_TO_BASKET);
                    }
                    break;
                case DRIVE_TO_SUBMERSIBLE:
                    if (timer.seconds() < 1.6) {
                        robot.drivetrain.profiledDriveToTarget(submersibleX, submersibleY + 27, submersibleHeading);
                    } else {
                        robot.lift.liftMode = Lift.LiftMode.GROUND;
                        robot.scoring.scoringMode = Scoring.ScoringMode.TRANSFER;
                        robot.drivetrain.profiledDriveToTarget(submersibleX, submersibleY, submersibleHeading);
                    }

                    if (timer.seconds() > 2.5) {
                        switchState(State.RETRIEVE_SUBMERSIBLE_SAMPLE);
                        strafe = true;
                    }
                    break;
                case RETRIEVE_SUBMERSIBLE_SAMPLE:
                    robot.scoring.scoringMode = Scoring.ScoringMode.SCORING_HIGH_CHAMBER;
                    if (timer.seconds() < 1.5 || robot.isDriving) {
                        robot.driveToHuskyLens();
                    }
                    if (timer.seconds() > 2.0 || !robot.isDriving) {
                        robot.submersibleIntake();
                        if(timer.seconds () > 2.3 || robot.extension.leftLink.getPosition() < 0.75) {
                            robot.intake.intakeMode = Intake.IntakeMode.INTAKE;
                        }
                        if (timer.seconds() > 4.0 || robot.intake.colors.green > 0.01) {
                            switchState(State.DRIVE_TO_BASKET_FROM_SUBMERSIBLE);
                            strafe = false;
                        } else if (robot.intake.hsvValues[0] > 100){
                            robot.intake.spinIntake.setPower(-1);
                        }
                    }
                    break;
                case DRIVE_TO_BASKET_FROM_SUBMERSIBLE:
                    robot.scoring.scoringMode = Scoring.ScoringMode.TRANSFER;
                    robot.extension.leftLink.setPosition(0.89);
                    robot.extension.rightLink.setPosition(0.89);
                    robot.intake.spinIntake.setPower(1);
                    robot.intake.intakeMode = Intake.IntakeMode.TRANSFER;
                    if (timer.seconds() > 1.7) {
                        robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                    }
                    if (timer.seconds() > 2.0) {
                        robot.lift.liftMode = Lift.LiftMode.HIGH_BASKET;
                        robot.scoring.scoringMode = Scoring.ScoringMode.SAMPLE;
                    }
                    robot.drivetrain.profiledDriveToTarget(basketX-1, basketY, basketHeading);
                    //put condition for switch at the end, condition can be based on time or completion of a task
                    if (timer.seconds() > 2.7) {
                        switchState(State.DELIVER_SAMPLE);
                    }
                    break;
                case PARK:
                    robot.drivetrain.profiledDriveToTarget(parkingX, parkingY, parkingHeading);
                    if (robot.drivetrain.targetReached || timer.seconds() > 2.0) {
                        switchState(State.IDLE);
                    }
                    break;
                case IDLE:
            }

            // Anything outside of the switch statement will run independent of the currentState
            // We update robot continuously in the background, regardless of state
            if (!strafe) {
                robot.update();
            }
            robot.lift.update();
            robot.scoring.update();
            robot.intake.update();
            robot.intake.sensorUpdate();

            telemetry.addData("state", currentState);
            telemetry.addData("timer", timer.seconds());
            telemetry.update();

        }
    }

    void switchState(State newState) {
        currentState = newState;
        robot.drivetrain.targetReached = false;
        timer.reset();
    }

}
