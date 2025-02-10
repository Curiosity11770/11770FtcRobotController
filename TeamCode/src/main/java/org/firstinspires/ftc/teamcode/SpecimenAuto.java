package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name="SpecimenAuto", group="Linear OpMode")
@Config
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
        SCORE,
        OPEN,
        BACK,
        FORWARD,
        PICKUP,
        DRIVE_BACK,
        LIFT2,
        DRIVE_SLOWLY2,
        SCORE2,
        OPEN2,
        BACK2,
        FORWARD2,
        PICKUP2,
        DRIVE_BACK2,
        LIFT3,
        DRIVE_SLOWLY3,
        SCORE3,
        OPEN3,
        PARK,
        IDLE

    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.DRIVE_TO_CHAMBER;

    // Define our start pose
    Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0);
    // Define our target
    public static double targetX = 23;
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

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState){
                case DRIVE_TO_CHAMBER:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.intake.flipIntake.setPosition(robot.intake.INTAKE_CLOSED);
                    robot.scoring.scoringPivot.setPosition(0);
                    if(robot.drivetrain.targetReached){
                        currentState = State.LIFT;
                        timer.reset();
                    }
                    break;
                case LIFT:
                    robot.scoring.scoringPivot.setPosition(0);
                    robot.lift.liftToPositionPIDClass(1400);
                    //robot.scoring.scoringPivot.setPosition(0.2);
                    if(timer.seconds() > 2.0){
                        currentState = State.DRIVE_SLOWLY;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 28.75, 5, AngleUnit.DEGREES, 0));
                        timer.reset();
                    }
                    break;
                case DRIVE_SLOWLY:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(robot.drivetrain.targetReached || timer.seconds() > 2){
                        currentState = State.SCORE;
                        timer.reset();
                    }
                    break;
                case SCORE:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //robot.lift.liftToPositionPIDClass(100);
                    robot.lift.liftToPositionPIDClass(600);
                    //robot.scoring.clawServo.setPosition(robot.scoring.CLAW_OPEN);
                    if(timer.seconds() > 1.2){
                        currentState = State.OPEN;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 10, 5, AngleUnit.DEGREES, 0));
                        timer.reset();
                    }
                    break;
                case OPEN:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_OPEN);
                    if(timer.seconds() > 1.5){
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
                    if(timer.seconds() > 2.0){
                        currentState = State.FORWARD;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 1, -32, AngleUnit.DEGREES, 0));
                        timer.reset();
                    }
                    break;
                case FORWARD:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //robot.scoring.clawServo.setPosition(0.8);
                    //robot.lift.liftToPositionPIDClass(0);
                    //robot.scoring.clawWrist.setPosition(0.5);
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_OPEN);
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.0){
                        currentState = State.PICKUP;
                        timer.reset();
                    }
                    break;
                case PICKUP:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_CLOSED);
                    if(timer.seconds() > 2.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 24, 12, AngleUnit.DEGREES, 0));
                        currentState = State.DRIVE_BACK;
                        timer.reset();
                    }
                    break;
                case DRIVE_BACK:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //robot.scoring.scoringPivot.setPosition(0.2);
                    robot.lift.liftToPositionPIDClass(1400);
                    robot.scoring.clawWrist.setPosition(robot.scoring.CLAW_UP);
                    robot.scoring.scoringPivot.setPosition(0);
                    if(robot.drivetrain.targetReached || timer.seconds() > 2.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 29.5, 12, AngleUnit.DEGREES, 0));
                        currentState = State.DRIVE_SLOWLY2;
                        timer.reset();
                    }
                    break;
                case DRIVE_SLOWLY2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.lift.liftToPositionPIDClass(700);
                    if(robot.drivetrain.targetReached || timer.seconds() > 2.0){
                        currentState = State.SCORE2;
                        timer.reset();
                    }
                    break;
                case SCORE2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //robot.scoring.scoringPivot.setPosition(0.2);
                    robot.lift.liftToPositionPIDClass(400);
                    if(timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 23, 12, AngleUnit.DEGREES, 0));
                        currentState = State.OPEN2;
                        timer.reset();
                    }
                    break;
                case OPEN2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_OPEN);
                    if(timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 6, -32, AngleUnit.DEGREES, 180));
                        currentState = State.BACK2;
                        timer.reset();
                    }
                    break;
                case BACK2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.lift.liftToPositionPIDClass(0);
                    //robot.scoring.scoringPivot.setPosition(robot.scoring.SCORING_UP);
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_CLOSED);
                    if(timer.seconds() > 2.0){
                        currentState = State.FORWARD2;
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 1, -32, AngleUnit.DEGREES, 0));
                        timer.reset();
                    }
                    break;
                case FORWARD2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //robot.scoring.clawServo.setPosition(0.8);
                    //robot.lift.liftToPositionPIDClass(0);
                    //robot.scoring.clawWrist.setPosition(0.5);
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_OPEN);
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.0){
                        currentState = State.PICKUP2;
                        timer.reset();
                    }
                    break;
                case PICKUP2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_CLOSED);
                    if(timer.seconds() > 2.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 24, 12, AngleUnit.DEGREES, 0));
                        currentState = State.DRIVE_BACK2;
                        timer.reset();
                    }
                    break;
                case DRIVE_BACK2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //robot.scoring.scoringPivot.setPosition(0.2);
                    robot.lift.liftToPositionPIDClass(1400);
                    robot.scoring.clawWrist.setPosition(robot.scoring.CLAW_UP);
                    robot.scoring.scoringPivot.setPosition(0);
                    if(robot.drivetrain.targetReached || timer.seconds() > 2.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 28.5, 12, AngleUnit.DEGREES, 0));
                        currentState = State.DRIVE_SLOWLY3;
                        timer.reset();
                    }
                    break;
                case DRIVE_SLOWLY3:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.lift.liftToPositionPIDClass(400);
                    if(robot.drivetrain.targetReached || timer.seconds() > 2.0){
                        currentState = State.SCORE3;
                        timer.reset();
                    }
                    break;
                case SCORE3:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //robot.scoring.scoringPivot.setPosition(0.2);
                    robot.lift.liftToPositionPIDClass(400);
                    if(timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 30.5, 12, AngleUnit.DEGREES, 0));
                        currentState = State.OPEN3;
                        timer.reset();
                    }
                    break;
                case OPEN3:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    //
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_OPEN);
                    if(timer.seconds() > 1.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 6, -32, AngleUnit.DEGREES, 180));
                        currentState = State.IDLE;
                        timer.reset();
                    }
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState
            // We update robot continuously in the background, regardless of state
            robot.update();

            telemetry.addData("state", currentState);
            telemetry.addData("position", targetPose);
            telemetry.update();

        }
    }

}
