package org.firstinspires.ftc.teamcode;

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

@Autonomous
public class SpecimenAuto extends LinearOpMode {

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
        PICKUP2,
        DRIVE_BACK2,
        LIFT3,
        DRIVE_SLOWLY3,
        OPEN3,
        BACK3,
        FORWARD3,
        PICKUP3,
        DRIVE_BACK4,
        LIFT4,
        DRIVE_SLOWLY4,
        OPEN4,
        BACK4,
        FORWARD4,
        PICKUP4,
        DRIVE_BACK5,
        LIFT5,
        DRIVE_SLOWLY5,
        OPEN5,
        PARK,
        IDLE

    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.DRIVE_TO_CHAMBER;

    // Define our start pose
    Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0);
    // Define our target
    public static double targetX = 7;
    public static double targetY = 5;
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
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.intake.intakeMode = Intake.IntakeMode.TRANSFER;
                    robot.scoring.scoringMode = Scoring.ScoringMode.SCORING_HIGH_CHAMBER;
                    robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                    robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                    robot.extension.leftLink.setPosition(0.87);
                    robot.extension.rightLink.setPosition(0.87);
                    if(robot.drivetrain.targetReached || timer.seconds() > 0.6){
                        currentState = State.DRIVE_SLOWLY;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 35, 5, AngleUnit.DEGREES, 0));
                        timer.reset();
                    }
                    break;
                case DRIVE_SLOWLY:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.5){
                        currentState = State.OPEN;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 11, 5, AngleUnit.DEGREES, 0));
                        timer.reset();
                    }
                    break;
                case OPEN:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_OPEN);
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 4, 4, AngleUnit.DEGREES, 0));
                        currentState = State.PREP;
                        timer.reset();
                    }
                    break;
                case PREP:
                    robot.scoring.scoringMode = Scoring.ScoringMode.PICKUP_WALL;
                    robot.lift.liftMode = Lift.LiftMode.WALL_PICKUP;
                    if(robot.drivetrain.targetReached || timer.seconds() > 0.75){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 4, -25, AngleUnit.DEGREES, 0));
                        currentState = State.STRAFE1;
                        timer.reset();
                    }
                    break;
                case STRAFE1:
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 55, -25, AngleUnit.DEGREES, 0));
                        currentState = State.UP;
                        timer.reset();
                    }
                    break;
                case UP:
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.5){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 55, -37, AngleUnit.DEGREES, 0));
                        currentState = State.STRAFE2;
                        timer.reset();
                    }
                    break;
                case STRAFE2:
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.5){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 6, -37, AngleUnit.DEGREES, 0));
                        currentState = State.IDLE;
                        timer.reset();
                    }
                    break;
                case STRAFE3:
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.5){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 6, -32, AngleUnit.DEGREES, 0));
                        currentState = State.BACK;
                        timer.reset();
                    }
                    break;
                case BACK:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.lift.liftToPositionPIDClass(0);
                    //robot.scoring.scoringPivot.setPosition(robot.scoring.SCORING_UP);
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_CLOSED);
                    if(timer.seconds() > 1.5){
                        currentState = State.FORWARD;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, -0.25, -32, AngleUnit.DEGREES, 0));
                        timer.reset();
                    }
                    break;
                case FORWARD:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //robot.scoring.clawServo.setPosition(0.8);
                    //robot.lift.liftToPositionPIDClass(0);
                    //robot.scoring.clawWrist.setPosition(0.5);
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_OPEN);
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.75){
                        currentState = State.PICKUP;
                        timer.reset();
                    }
                    break;
                case PICKUP:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_CLOSED);
                    robot.lift.liftToPositionPIDClass(1300);
                    if(timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 11, 8, AngleUnit.DEGREES, 0));
                        currentState = State.DRIVE_BACK;
                        timer.reset();
                    }
                    break;
                case DRIVE_BACK:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //robot.scoring.scoringPivot.setPosition(0.2);
                    robot.scoring.scoringPivot.setPosition(0);
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 36, 8, AngleUnit.DEGREES, 0));
                        currentState = State.DRIVE_SLOWLY2;
                        timer.reset();
                    }
                    break;
                case DRIVE_SLOWLY2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.5){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 6, 8, AngleUnit.DEGREES, 0));
                        currentState = State.OPEN2;
                        timer.reset();
                    }
                    break;
                case OPEN2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_OPEN);
                    if(timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 6, -32, AngleUnit.DEGREES, 0));
                        currentState = State.BACK2;
                        timer.reset();
                    }
                    break;
                case BACK2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.lift.liftToPositionPIDClass(0);
                    //robot.scoring.scoringPivot.setPosition(robot.scoring.SCORING_UP);
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_CLOSED);
                    if(timer.seconds() > 1.5){
                        currentState = State.FORWARD2;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, -0.25, -32, AngleUnit.DEGREES, 0));
                        timer.reset();
                    }
                    break;
                case FORWARD2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //robot.scoring.clawServo.setPosition(0.8);
                    //robot.lift.liftToPositionPIDClass(0);
                    //robot.scoring.clawWrist.setPosition(0.5);
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_OPEN);
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.75){
                        currentState = State.PICKUP2;
                        timer.reset();
                    }
                    break;
                case PICKUP2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_CLOSED);
                    robot.lift.liftToPositionPIDClass(1300);
                    if(timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 8, 11, AngleUnit.DEGREES, 0));
                        currentState = State.DRIVE_BACK2;
                        timer.reset();
                    }
                    break;
                case DRIVE_BACK2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //robot.scoring.scoringPivot.setPosition(0.2);
                    robot.scoring.scoringPivot.setPosition(0);
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.5){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 36, 11, AngleUnit.DEGREES, 0));
                        currentState = State.DRIVE_SLOWLY3;
                        timer.reset();
                    }
                    break;
                case DRIVE_SLOWLY3:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.5){
                        currentState = State.OPEN3;
                        timer.reset();
                    }
                    break;
                case OPEN3:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_OPEN);
                    if(timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 6, 0, AngleUnit.DEGREES, 0));
                        currentState = State.IDLE;
                        timer.reset();
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
