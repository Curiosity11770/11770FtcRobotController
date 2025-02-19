package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Autonomous(name="RegionalsSample", group="Linear OpMode")
@Config
public class RegionalsSample extends LinearOpMode {

    Robot robot;

    ElapsedTime timer = new ElapsedTime();

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    //TODO Update states to reflect flow of robot actions
    enum State {
        DRIVE_TO_BASKET,
        GO,
        SCORE,
        RUN,
        SAMPLE1,
        PICKUP,
        TRANSFER,
        GRAB,
        UP,
        GO2,
        SCORE2,
        IDLE


    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.DRIVE_TO_BASKET;

    // Define our start pose
    Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0);
    // Define our target
    public static double targetX = 14;
    public static double targetY = 8;
    public static double targetHeading = -45;
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
        timer.reset();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState){
                case DRIVE_TO_BASKET:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.intake.intakeMode = robot.intake.intakeMode.TRANSFER;
                    robot.scoring.scoringMode = robot.scoring.scoringMode.SAMPLE;
                    robot.lift.liftMode = robot.lift.liftMode.HIGH_BASKET;
                    if(robot.drivetrain.targetReached || timer.seconds() > 3.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 1.2, 17.25, AngleUnit.DEGREES, -45));
                        currentState = State.GO;
                        timer.reset();
                    }
                    break;
                case GO:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    if(timer.seconds() > 1.5){
                        currentState = State.SCORE;
                        timer.reset();
                    }

                    break;
                case SCORE:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_OPEN);
                    if(robot.drivetrain.targetReached || timer.seconds() > 0.75){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 1.5, 9 , AngleUnit.DEGREES, 0));
                        currentState = State.RUN;
                        timer.reset();
                    }
                    break;
                case RUN:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    robot.intake.intakeMode = robot.intake.intakeMode.INTAKE;
                    robot.lift.liftMode = robot.lift.liftMode.GROUND;
                    robot.extension.leftLink.setPosition(0);
                    if(robot.drivetrain.targetReached || timer.seconds() > 1){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 1, 7, AngleUnit.DEGREES, 0));
                        currentState = State.SAMPLE1;
                        timer.reset();
                    }
                    break;
                case SAMPLE1:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    robot.scoring.scoringMode = robot.scoring.scoringMode.TRANSFER;
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_OPEN);
                    robot.extension.leftLink.setPosition(0.9);
                    robot.intake.spinIntake.setPower(1);
                    robot.lift.liftMode = robot.lift.liftMode.GROUND;
                    if(timer.seconds() > 1.5){
                        currentState = State.PICKUP;
                        timer.reset();
                    }
                    break;
                case PICKUP:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    robot.intake.spinIntake.setPower(1);
                    robot.extension.leftLink.setPosition(0.9);
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_CLOSED);
                    robot.lift.liftMode = robot.lift.liftMode.GROUND;
                    if(timer.seconds() > 3.0){
                        currentState = State.UP;
                        timer.reset();
                    }
                    break;
                case UP:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    //robot.scoring.scoringPivot.setMode
                    robot.lift.liftMode = robot.lift.liftMode.HIGH_BASKET;
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.5){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 1.2, 17.25, AngleUnit.DEGREES, -45));
                        currentState = State.GO2;
                        timer.reset();
                    }
                    break;
                case GO2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.5){
                        currentState = State.SCORE2;
                        timer.reset();
                    }
                    break;
                case SCORE2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServo.setPosition(robot.scoring.CLAW_OPEN);
                    if(timer.seconds() > 0.75){
                        //robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 1.5, 7, AngleUnit.DEGREES, 0));
                        currentState = State.IDLE;
                        timer.reset();
                    }
                    break;

            }
            // Anything outside of the switch statement will run independent of the currentState
            // We update robot continuously in the background, regardless of state
            robot.update();
            robot.lift.update();
            robot.scoring.update();
            robot.intake.update();

            telemetry.addData("state", currentState);
            telemetry.addData("position", targetPose);
            telemetry.update();

        }
    }

}


