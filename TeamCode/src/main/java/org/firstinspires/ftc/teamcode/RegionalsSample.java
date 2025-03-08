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
        UP,
        GO2,
        SCORE2,
        RUN2,
        SAMPLE2,
        PICKUP2,
        UP2,
        GO3,
        SCORE3,
        RUN3,
        SAMPLE3,
        PICKUP3,
        UP3,
        GO4,
        SCORE4,
        SUBMERSIBLE,
        ALIGN,
        INTAKE,
        TRANSFER,
        UP5,
        GO5,
        SCORE5,
        IDLE


    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.DRIVE_TO_BASKET;
    Boolean strafe  = false;

    // Define our start pose
    Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0);
    // Define our target
    public static double targetX = 9;
    public static double targetY = 1;
    public static double targetHeading = -45;
    Pose2D targetPose = new Pose2D(DistanceUnit.INCH, targetX,targetY, AngleUnit.DEGREES, targetHeading);

    @Override
    public void  runOpMode() {
        //calling constructor
        robot = new Robot(this);


        //calling init function
        robot.init();

        //TODO Pass starting pose to localizer
        //for Gobilda it looks like this
        robot.drivetrain.localizer.odo.setPosition(startPose);
        //for sparkfun it looks like this
        // robot.drivetrain.localizer.myOtos.s etPosition(startPose);

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
                    robot.intake.intakeMode = Intake.IntakeMode.TRANSFER;
                    robot.scoring.scoringMode = Scoring.ScoringMode.SAMPLE;
                    robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                    robot.lift.liftMode = Lift.LiftMode.HIGH_BASKET;
                    if(robot.drivetrain.targetReached || timer.seconds() > 2.0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 1.2, 17.25, AngleUnit.DEGREES, -65));
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
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    if(robot.drivetrain.targetReached || timer.seconds() > 0.2){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 12.5, 7, AngleUnit.DEGREES, 0));
                        currentState = State.RUN;
                        timer.reset();
                    }
                    break;
                case RUN:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    robot.intake.intakeMode = Intake.IntakeMode.INTAKE;
                    robot.extension.leftLink.setPosition(0.5);
                    robot.extension.rightLink.setPosition(0.5);
                    robot.extension.leftLinkPosition = 0.2;
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.0){
                        currentState = State.SAMPLE1;
                        timer.reset();
                    }
                    break;
                case SAMPLE1:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    double constant = 0.003;
                    robot.extension.leftLink.setPosition(robot.extension.leftLinkPosition);
                    robot.extension.rightLink.setPosition(robot.extension.leftLinkPosition);
                    robot.extension.leftLinkPosition -= constant;
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    robot.intake.spinIntake.setPower(1);
                    robot.lift.liftMode = Lift.LiftMode.GROUND;
                    robot.scoring.scoringMode = Scoring.ScoringMode.TRANSFER;
                    if(timer.seconds() > 0.5){
                        currentState = State.PICKUP;
                        timer.reset();
                    }
                    break;
                case PICKUP:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    robot.scoring.scoringMode = Scoring.ScoringMode.TRANSFER;
                    robot.intake.intakeMode = Intake.IntakeMode.TRANSFER;
                    robot.intake.spinIntake.setPower(1);
                    robot.extension.leftLink.setPosition(0.87);
                    robot.extension.rightLink.setPosition(0.87);
                    if(timer.seconds() > 1.2){
                        robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                        if (timer.seconds() > 1.3){
                            currentState = State.UP;
                            timer.reset();
                        }
                    }
                    break;
                case UP:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    //robot.scoring.scoringPivot.setMode
                    robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                    robot.lift.liftMode = Lift.LiftMode.HIGH_BASKET;
                    robot.scoring.scoringMode = Scoring.ScoringMode.SAMPLE;
                if(robot.drivetrain.targetReached || timer.seconds() > 1.25){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 1.2, 17.25, AngleUnit.DEGREES, -65));
                        currentState = State.GO2;
                        timer.reset();
                    }
                    break;
                case GO2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    if(robot.drivetrain.targetReached || timer.seconds() > 0.5){
                        currentState = State.SCORE2;
                        timer.reset();
                    }
                    break;
                case SCORE2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    robot.intake.spinIntake.setPower(1);
                    if(robot.drivetrain.targetReached || timer.seconds() > 0.2){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 11.5, 20.5, AngleUnit.DEGREES, 2));
                        currentState = State.RUN2;
                        timer.reset();
                    }
                    break;
                case RUN2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    robot.intake.intakeMode = Intake.IntakeMode.INTAKE;
                    robot.extension.leftLink.setPosition(0.5);
                    robot.extension.rightLink.setPosition(0.5);
                    robot.extension.leftLinkPosition = 0.2;
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.0){
                        currentState = State.SAMPLE2;
                        timer.reset();
                    }
                    break;
                case SAMPLE2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    double constant2 = 0.003;
                    robot.extension.leftLink.setPosition(robot.extension.leftLinkPosition);
                    robot.extension.rightLink.setPosition(robot.extension.leftLinkPosition);
                    robot.extension.leftLinkPosition -= constant2;
                    robot.scoring.scoringMode = Scoring.ScoringMode.TRANSFER;
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    robot.intake.spinIntake.setPower(1);
                    robot.lift.liftMode = Lift.LiftMode.GROUND;
                    if(timer.seconds() > 0.75){
                        currentState = State.PICKUP2;
                        timer.reset();
                    }
                    break;
                case PICKUP2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    robot.intake.intakeMode = Intake.IntakeMode.TRANSFER;
                    robot.intake.spinIntake.setPower(1);
                    robot.extension.leftLink.setPosition(0.87);
                    robot.extension.rightLink.setPosition(0.87);
                    robot.lift.liftMode = Lift.LiftMode.GROUND;
                    if(timer.seconds() > 1.0){
                        robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                        if (timer.seconds() > 1.2 ){
                            currentState = State.UP2;
                            timer.reset();
                        }
                    }
                    break;
                case UP2:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    //robot.scoring.scoringPivot.setMode
                    robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                    robot.lift.liftMode = Lift.LiftMode.HIGH_BASKET;
                    robot.scoring.scoringMode = Scoring.ScoringMode.SAMPLE;
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.25){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 1.2, 17.25, AngleUnit.DEGREES, -65));
                        currentState = State.GO3;
                        timer.reset();
                    }
                    break;
                case GO3:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.5){
                        currentState = State.SCORE3;
                        timer.reset();
                    }
                    break;
                case SCORE3:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    robot.intake.spinIntake.setPower(1);
                    if(timer.seconds() > 0.5){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 22, 7, AngleUnit.DEGREES, 55));
                        currentState = State.RUN3;
                        timer.reset();
                    }
                    break;
                case RUN3:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    robot.intake.intakeMode = Intake.IntakeMode.INTAKE;
                    robot.extension.leftLink.setPosition(0.35);
                    robot.extension.rightLink.setPosition(0.35);
                    robot.extension.leftLinkPosition = 0.35;
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.0){
                        currentState = State.SAMPLE3;
                        timer.reset();
                    }
                    break;
                case SAMPLE3:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    double constant3 = 0.003;
                    robot.extension.leftLink.setPosition(robot.extension.leftLinkPosition);
                    robot.extension.rightLink.setPosition(robot.extension.leftLinkPosition);
                    robot.extension.leftLinkPosition -= constant3;
                    robot.scoring.scoringMode = Scoring.ScoringMode.TRANSFER;
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    robot.intake.spinIntake.setPower(1);
                    robot.lift.liftMode = Lift.LiftMode.GROUND;
                    if(timer.seconds() > 1.0){
                        currentState = State.PICKUP3;
                        timer.reset();
                    }
                    break;
                case PICKUP3:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    robot.intake.intakeMode = Intake.IntakeMode.TRANSFER;
                    robot.intake.spinIntake.setPower(1);
                    robot.extension.leftLink.setPosition(0.87);
                    robot.extension.rightLink.setPosition(0.87);
                    robot.lift.liftMode = Lift.LiftMode.GROUND;
                    if(timer.seconds() > 1.0){
                        robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                        if (timer.seconds() > 1.2){
                            currentState = State.UP3;
                            timer.reset();
                        }
                    }
                    break;
                case UP3:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    //robot.scoring.scoringPivot.setMode
                    robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                    robot.lift.liftMode = Lift.LiftMode.HIGH_BASKET;
                    robot.scoring.scoringMode = Scoring.ScoringMode.SAMPLE;
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.25){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 1.2, 17.25, AngleUnit.DEGREES, -65));
                        currentState = State.GO4;
                        timer.reset();
                    }
                    break;
                case GO4:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task;
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.0){
                        currentState = State.SCORE4;
                        timer.reset();
                    }
                    break;
                case SCORE4:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    if(timer.seconds() > 0.2){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 64, -28, AngleUnit.DEGREES, -90));
                        currentState = State.SUBMERSIBLE;
                        timer.reset();
                    }
                    break;
                case SUBMERSIBLE:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    if(timer.seconds() > 1.0){
                        robot.lift.liftMode = Lift.LiftMode.GROUND;
                        robot.scoring.scoringMode = Scoring.ScoringMode.TRANSFER;
                    }
                    if(timer.seconds() > 3.0){
                        //robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 1.5, 7, AngleUnit.DEGREES, 0));
                        currentState = State.ALIGN;
                        strafe = true;
                        timer.reset();
                    }
                    break;
                case ALIGN:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    while(timer.seconds() < 3.0) {
                        robot.drivetrain.localizer.update();
                        robot.driveToHuskyLens();
                        telemetry.addData("while loop", currentState);
                    }
                    if(timer.seconds() > 3.0){
                        //robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 1.5, 7, AngleUnit.DEGREES, 0));
                        currentState = State.INTAKE;
                        strafe = true;
                        timer.reset();
                    }
                    break;
                case INTAKE:
                    //put condition for switch at the beginning, condition can be based on time or completion of a task
                    while(timer.seconds() < 3.0){
                        robot.drivetrain.localizer.update();
                        robot.submersibleIntake();
                        telemetry.addData("while loop", currentState);
                        }
                    if(timer.seconds() > 3.0 || robot.intake.colors.green > 0.01){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 1.2, 17.25, AngleUnit.DEGREES, 0));
                        currentState = State.UP5;
                        strafe = false;
                        timer.reset();
                    }
                    break;
                case UP5:
                    if(timer.seconds() < 1.2) {
                        robot.extension.leftLink.setPosition(0.87);
                        robot.extension.rightLink.setPosition(0.87);
                        robot.intake.intakeMode = Intake.IntakeMode.INTAKE;
                    }else if(timer.seconds() < 1.3 && timer.seconds() > 1.2) {
                        robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                    } else if (timer.seconds() > 1.3){
                        robot.scoring.scoringMode = Scoring.ScoringMode.SAMPLE;
                        robot.lift.liftMode = Lift.LiftMode.HIGH_BASKET;
                    }

                    if(timer.seconds() > 2.5){
                        currentState = State.SCORE5;
                        timer.reset();
                    }
                case SCORE5:
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    if(timer.seconds() > 1.0){
                        currentState = State.IDLE;
                        timer.reset();
                    }
                    break;



            }
            // Anything outside of the switch statement will run independent of the currentState
            // We update robot continuously in the background, regardless of state
            if (!strafe){
            robot.update();
            }
            robot.lift.update();
            robot.scoring.update();
            robot.intake.update();

            telemetry.addData("state", currentState);
            telemetry.addData("position", targetPose);
            telemetry.update();

        }
    }

}


