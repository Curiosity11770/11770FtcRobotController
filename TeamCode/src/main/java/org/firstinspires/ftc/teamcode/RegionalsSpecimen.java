package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Scoring;

@Autonomous
@Config
public class RegionalsSpecimen extends LinearOpMode {

    Robot robot;

    ElapsedTime timer = new ElapsedTime();

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    //TODO Update states to reflect flow of robot actions
    enum State {
        DRIVE_TO_CHAMBER,
        LIFT,
        DRIVE_SLOWLY,
        OPEN,
        PREP,
        GO,
        STRAFE1,
        UP,
        BACK,
        STRAFE2,
        UP2,
        BACK2,
        STRAFE3,
        UP3,
        STRAFE4,
        FORWARD,
        PICKUP,
        DRIVE_BACK,
        LIFT2,
        DRIVE_SLOWLY2,
        OPEN2,
        BACK1,
        FORWARD2,
        SLOW,
        PICKUP2,
        DRIVE_BACK2,
        LIFT3,
        DRIVE_SLOWLY3,
        FORWARD3,
        SLOW2,
        SLOW3,
        PICKUP3,
        DRIVE_BACK3,
        BACK3,
        BACK4,
        DRIVE_SLOWLY4,
        FORWARD4,
        SLOW4,
        PICKUP4,
        DRIVE_BACK4,
        IDLE

    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.DRIVE_TO_CHAMBER;

    // Define our start pose
    Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0);
    // Define our target
    public static double targetX = 14;
    public static double targetY = 10;
    public static double targetHeading = 0;
    Pose2D targetPose = new Pose2D(DistanceUnit.INCH, targetX,targetY, AngleUnit.DEGREES, targetHeading);

    @Override
    public void runOpMode() {
        //calling constructor
        robot = new Robot(this);


        //calling init function
        robot.init();

        //TODO Pass starting pose to localizer
        //for Gobilda it looks like this
        robot.drivetrain.localizer.odo.setPosition(startPose);
        //for sparkfun it looks like this
        // robot.drivetrain.localizer.myOtos.setPosition(startPose);

        //Set the drivetrain's first target
        robot.drivetrain.setTargetPose(targetPose);
        robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState){
                case DRIVE_TO_CHAMBER:
                    robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                    robot.intake.intakeMode = Intake.IntakeMode.TRANSFER;
                    robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                    if(robot.drivetrain.targetReached || timer.seconds() > 0.8){
                        currentState = State.LIFT;
                       timer.reset();
                    }
                    break;
                case LIFT:
                        robot.scoring.scoringMode = Scoring.ScoringMode.SCORING_HIGH_CHAMBER;
                        robot.extension.leftLink.setPosition(0.87);
                        robot.extension.rightLink.setPosition(0.87);
                    if(timer.seconds() > 0.4){
                        currentState = State.DRIVE_SLOWLY;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 38, 10, AngleUnit.DEGREES, 0));
                        timer.reset();
                    }
                    break;
                case DRIVE_SLOWLY:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.1){
                        currentState = State.OPEN;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 11, 5, AngleUnit.DEGREES, 0));
                        timer.reset();
                    }
                    break;
                case OPEN:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    if(robot.drivetrain.targetReached || timer.seconds() > 0.5){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 4, 4, AngleUnit.DEGREES, 0));
                        currentState = State.PREP;
                        timer.reset();
                    }
                    break;
                case PREP:
                    robot.scoring.scoringMode = Scoring.ScoringMode.PICKUP_WALL;
                    robot.lift.liftMode = Lift.LiftMode.WALL_PICKUP;
                    if(robot.drivetrain.targetReached || timer.seconds() > 0.6){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 4, -19, AngleUnit.DEGREES, 0));
                        currentState = State.GO    ;
                        timer.reset();
                    }
                    break;
                case GO:
                    robot.scoring.scoringMode = Scoring.ScoringMode.PICKUP_WALL;
                    robot.lift.liftMode = Lift.LiftMode.WALL_PICKUP;
                    if(robot.drivetrain.targetReached || timer.seconds() > 0.75){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 50, -19, AngleUnit.DEGREES, 0));
                        currentState = State.STRAFE1;
                        timer.reset();
                    }
                    break;
                case STRAFE1:
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 50, -31, AngleUnit.DEGREES, 0));
                        currentState = State.UP;
                        timer.reset();
                    }
                    break;
                case UP:
                    if(robot.drivetrain.targetReached || timer.seconds() > 0.5){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 12, -31, AngleUnit.DEGREES, 0));
                        currentState = State.BACK;
                        timer.reset();
                    }
                    break;
                case BACK:
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.2){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 50, -31, AngleUnit.DEGREES, 0));
                        currentState = State.STRAFE2;
                        timer.reset();
                    }
                    break;
                case STRAFE2:
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 50, -40, AngleUnit.DEGREES, 0));
                        currentState = State.UP2;
                        timer.reset();
                    }
                    break;
                case UP2:
                    if(robot.drivetrain.targetReached || timer.seconds() > 0.5){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 12, -40, AngleUnit.DEGREES, 0));
                        currentState = State.BACK2;
                        timer.reset();
                    }
                    break;
                case BACK2:
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.25){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 50, -40, AngleUnit.DEGREES, 0));
                        currentState = State.STRAFE3;
                        timer.reset();
                    }
                    break;
                case STRAFE3:
                    if(robot.drivetrain.targetReached || timer.seconds() > 0.35){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 50, -47.5, AngleUnit.DEGREES, 0));
                        currentState = State.UP3;
                        timer.reset();
                    }
                    break;
                case UP3:
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 12, -47.5, AngleUnit.DEGREES, 0));
                        currentState = State.STRAFE4;
                        timer.reset();
                        }
                        break;
                case STRAFE4:
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.5){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 4, -30.5, AngleUnit.DEGREES, 0));
                        currentState = State.FORWARD;
                        timer.reset();
                    }
                    break;
                case FORWARD:
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    if(timer.seconds() > 1){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 0.2, -30.5, AngleUnit.DEGREES, 0));
                        currentState = State.SLOW;
                        timer.reset();
                    }
                    break;
                case SLOW:
                    if(timer.seconds() < 0.3) {
                        robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                        robot.scoring.scoringMode = Scoring.ScoringMode.PICKUP_WALL;
                        robot.lift.liftMode = Lift.LiftMode.WALL_PICKUP;
                    }
                    if(timer.seconds() > 0.3){
                        robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                        if(timer.seconds() > 0.4){
                            robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                        }
                        if(timer.seconds() > 0.6){
                            currentState = State.PICKUP;
                            timer.reset();
                        }
                    }
                    break;
                case PICKUP:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                    robot.scoring.scoringMode = Scoring.ScoringMode.SCORING_HIGH_CHAMBER;
                    robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                    if(timer.seconds() > 0.1){
                        robot.scoring.scoringMode = Scoring.ScoringMode.SCORING_HIGH_CHAMBER;
                        robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                        if(timer.seconds() > 0.2) {
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 14, 15, AngleUnit.DEGREES, 0));
                            currentState = State.DRIVE_BACK;
                            timer.reset();
                        }
                    }
                    break;
                case DRIVE_BACK:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 36, 15, AngleUnit.DEGREES, 0));
                        currentState = State.DRIVE_SLOWLY2;
                        timer.reset();
                    }
                    break;
                case DRIVE_SLOWLY2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(timer.seconds() > 1.25){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 15, 15, AngleUnit.DEGREES, 0));
                        currentState = State.BACK1;
                        timer.reset();
                    }
                    break;
                case BACK1:
                    robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                    robot.scoring.scoringMode = Scoring.ScoringMode.SCORING_HIGH_CHAMBER;
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    if(timer.seconds() > 0.5){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 4, -30.5, AngleUnit.DEGREES, 0));
                        currentState = State.FORWARD2;
                        timer.reset();
                    }
                    break;
                case FORWARD2:
                    robot.lift.liftMode = Lift.LiftMode.WALL_PICKUP;
                    robot.scoring.scoringMode = Scoring.ScoringMode.PICKUP_WALL;
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.65) {
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 0.2, -30.5, AngleUnit.DEGREES, 0));
                        currentState = State.SLOW2;
                        timer.reset();
                    }
                break;
                case SLOW2:
                    if(timer.seconds() < 0.3) {
                        robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                        robot.scoring.scoringMode = Scoring.ScoringMode.PICKUP_WALL;
                        robot.lift.liftMode = Lift.LiftMode.WALL_PICKUP;
                    }
                    if(timer.seconds() > 0.3){
                        robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                        if(timer.seconds() > 0.4){
                            robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                        }
                        if(timer.seconds() > 0.6){
                            currentState = State.PICKUP2;
                            timer.reset();
                        }
                    }
                    break;
                case PICKUP2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                    robot.scoring.scoringMode = Scoring.ScoringMode.SCORING_HIGH_CHAMBER;
                    robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                    if(timer.seconds() > 0.1){
                        robot.scoring.scoringMode = Scoring.ScoringMode.SCORING_HIGH_CHAMBER;
                        robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                        if(timer.seconds() > 0.2) {
                            robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 14, 19, AngleUnit.DEGREES, 0));
                            currentState = State.DRIVE_BACK3;
                            timer.reset();
                        }
                    }
                    break;
                case DRIVE_BACK3:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 36, 19, AngleUnit.DEGREES, 0));
                        currentState = State.BACK3;
                        timer.reset();
                    }
                    break;
                case BACK3:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(timer.seconds() > 1.2){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 15, 19, AngleUnit.DEGREES, 0));
                        currentState = State.DRIVE_SLOWLY3;
                        timer.reset();
                    }
                    break;
                case DRIVE_SLOWLY3:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    if(timer.seconds() > 0.5){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 4, -30.5, AngleUnit.DEGREES, 0));
                        currentState = State.FORWARD3;
                        timer.reset();
                    }
                    break;
                case FORWARD3:
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    robot.lift.liftMode = Lift.LiftMode.WALL_PICKUP;
                    robot.scoring.scoringMode = Scoring.ScoringMode.PICKUP_WALL;
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.65) {
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 0.3, -30.5, AngleUnit.DEGREES, 0));
                        currentState = State.SLOW3;
                        timer.reset();
                    }
                    break;
                case SLOW3:
                    if(timer.seconds() < 0.3) {
                        robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                        robot.scoring.scoringMode = Scoring.ScoringMode.PICKUP_WALL;
                        robot.lift.liftMode = Lift.LiftMode.WALL_PICKUP;
                    }
                    if(timer.seconds() > 0.3){
                        robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                        if(timer.seconds() > 0.4){
                            robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                        }
                        if(timer.seconds() > 0.6){
                            currentState = State.PICKUP3;
                            timer.reset();
                        }
                    }
                    break;
                case PICKUP3:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                    robot.scoring.scoringMode = Scoring.ScoringMode.SCORING_HIGH_CHAMBER;
                    robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                    if(timer.seconds() > 0.2) {
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 14, 13, AngleUnit.DEGREES, 0));
                        currentState = State.DRIVE_BACK2;
                        timer.reset();
                    }
                    break;
                case DRIVE_BACK2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 36, 13, AngleUnit.DEGREES, 0));
                        currentState = State.BACK4;
                        timer.reset();
                    }
                    break;
                case BACK4:

                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(timer.seconds() > 1.25){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 15, 13, AngleUnit.DEGREES, 0));
                        currentState = State.DRIVE_SLOWLY4;
                        timer.reset();
                    }
                    break;
                case DRIVE_SLOWLY4:
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(timer.seconds() > 0.5){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 4, -30.5, AngleUnit.DEGREES, 0));
                        currentState = State.FORWARD4;
                        timer.reset();
                    }
                    break;
                case FORWARD4:
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    robot.lift.liftMode = Lift.LiftMode.WALL_PICKUP;
                    robot.scoring.scoringMode = Scoring.ScoringMode.PICKUP_WALL;
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.65) {
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 0.2, -30.5, AngleUnit.DEGREES, 0));
                        currentState = State.SLOW4;
                        timer.reset();
                    }
                    break;
                case SLOW4:
                    if(timer.seconds() < 0.3) {
                        robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                        robot.scoring.scoringMode = Scoring.ScoringMode.PICKUP_WALL;
                        robot.lift.liftMode = Lift.LiftMode.WALL_PICKUP;
                    }
                    if(timer.seconds() > 0.3){
                        robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                        if(timer.seconds() > 0.4){
                            robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                        }
                        if(timer.seconds() > 0.6){
                            currentState = State.PICKUP2;
                            timer.reset();
                        }
                    }
                case PICKUP4:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                    robot.scoring.scoringMode = Scoring.ScoringMode.SCORING_HIGH_CHAMBER;
                    robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                    if(timer.seconds() > 0.2) {
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 14, 7, AngleUnit.DEGREES, 0));
                        currentState = State.DRIVE_BACK4;
                        timer.reset();
                    }
                    break;
                case DRIVE_BACK4:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 36, 7, AngleUnit.DEGREES, 0));
                        if(timer.seconds() > 1.2) {
                            robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                            currentState = State.IDLE;
                            timer.reset();
                        }

                    }
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState
            // We update robot continuously in the background, regardless of state
            robot.update();
            robot.scoring.update();
            robot.lift.update();
            robot.intake.update();
            telemetry.addData("state", currentState);
            telemetry.addData("position", targetPose);
            telemetry.update();

        }
    }

}


