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
@Autonomous(name="WorldsSample", group="Linear OpMode")
public class WorldsSample extends LinearOpMode {

    Robot robot;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime totalTime = new ElapsedTime();

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

    public static double submersibleX = 48;
    public static double submersibleY = -12;
    public static double submersibleHeading = -90;

    public static double parkingX = 48;
    public static double parkingY = -20;
    public static double parkingHeading = -90;

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.DRIVE_TO_BASKET;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0);

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
            switch (currentState){
                case DRIVE_TO_BASKET:
                    robot.intake.spinIntake.setPower(0);
                    robot.drivetrain.profiledDriveToTarget(basketX, basketY,basketHeading);
                    robot.lift.liftMode = Lift.LiftMode.HIGH_BASKET;
                    robot.scoring.scoringMode = Scoring.ScoringMode.SAMPLE;
                    //put condition for switch at the end, condition can be based on time or completion of a task
                    if(timer.seconds() > 1.5){
                        switchState(State.DELIVER_SAMPLE);
                    }
                    break;
                case DELIVER_SAMPLE:
                    robot.drivetrain.relativeDriveToTarget(-6, 0, 0, 0.1);
                    if (timer.seconds() > 1) {
                        robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    }
                    if(timer.seconds() > 1.2 && samplesScored < 3){
                        switchState(State.DRIVE_TO_FLOOR_SAMPLE);
                        samplesScored++;
                    }else if(timer.seconds() >1.2){
                        switchState(State.DRIVE_TO_SUBMERSIBLE);
                        samplesScored++;
                    }
                    break;
                case DRIVE_TO_FLOOR_SAMPLE:
                    if(timer.seconds() > 0.8) {
                        robot.lift.liftMode = Lift.LiftMode.GROUND;
                        robot.scoring.scoringMode = Scoring.ScoringMode.TRANSFER;
                    }
                    robot.extension.leftLink.setPosition(0.55);
                    robot.extension.rightLink.setPosition(0.55);
                    robot.intake.intakeMode = Intake.IntakeMode.INTAKE;
                    robot.intake.spinIntake.setPower(1);
                    telemetry.addData("expression", floorSampleY+samplesScored*6.0);
                    if(samplesScored == 1) {
                        robot.drivetrain.profiledDriveToTarget(floorSampleX, floorSampleY,floorSampleHeading+(samplesScored-1)*20.0);

                    } else if (samplesScored == 2){
                        robot.drivetrain.profiledDriveToTarget(floorSampleX, floorSampleY,2);
                    } else if (samplesScored == 3) {
                        robot.drivetrain.profiledDriveToTarget(floorSampleX, 19,floorSampleHeading+(samplesScored-1)*21);

                    }
                     if(timer.seconds() > 1.5){
                        switchState(State.RETRIEVE_FLOOR_SAMPLE);
                    }
                    break;
                case RETRIEVE_FLOOR_SAMPLE:
                    robot.drivetrain.relativeDriveToTarget(12, 0, 0, 0.03);
                    if(timer.seconds() > 1.5 || robot.intake.colors.green > 0.01){
                        switchState(State.TRANSFER);
                    }
                    break;
                case TRANSFER:
                    robot.extension.leftLink.setPosition(0.87);
                    robot.extension.rightLink.setPosition(0.87);
                    robot.intake.spinIntake.setPower(1);
                    robot.intake.intakeMode = Intake.IntakeMode.TRANSFER;
                    robot.drivetrain.profiledDriveToTarget(basketX, basketY,basketHeading);
                    if(timer.seconds() > 0.8){
                        robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                    }
                    if(timer.seconds() > 1.0){
                        switchState(State.DRIVE_TO_BASKET);
                    }
                    break;
                case DRIVE_TO_SUBMERSIBLE:
                    if(timer.seconds() <1.6 ){
                        robot.drivetrain.profiledDriveToTarget(submersibleX, submersibleY+12,submersibleHeading);
                    }else{
                        robot.drivetrain.profiledDriveToTarget(submersibleX, submersibleY,submersibleHeading);
                    }

                    if(timer.seconds() > 2.5){
                        switchState(State.RETRIEVE_SUBMERSIBLE_SAMPLE);
                    }
                    break;
                case RETRIEVE_SUBMERSIBLE_SAMPLE:
                    if(timer.seconds() < 2.0) {
                        robot.driveToHuskyLens();
                    }
                    if(timer.seconds() > 2.0){
                        robot.submersibleIntake();
                        if(timer.seconds() > 3.0) {
                            switchState(State.DRIVE_TO_BASKET_FROM_SUBMERSIBLE);
                        }
                    }
                    break;
                case DRIVE_TO_BASKET_FROM_SUBMERSIBLE:
                    robot.drivetrain.profiledDriveToTarget(basketX, basketY,basketHeading);
                    //put condition for switch at the end, condition can be based on time or completion of a task
                    if(robot.drivetrain.targetReached || timer.seconds() > 3){
                        switchState(State.DELIVER_SAMPLE);
                    }
                    break;
                case PARK:
                    robot.drivetrain.profiledDriveToTarget(parkingX, parkingY,parkingHeading);
                    if(robot.drivetrain.targetReached || timer.seconds() > 2.0){
                        switchState(State.IDLE);
                    }
                    break;
                case IDLE:
            }

            // Anything outside of the switch statement will run independent of the currentState
            // We update robot continuously in the background, regardless of state
            robot.update();
            robot.lift.update();
            robot.scoring.update();
            robot.intake.update();
            robot.intake.sensorUpdate();

            telemetry.addData("state", currentState);
            telemetry.addData("timer", timer.seconds());
            telemetry.update();

        }
    }

    void switchState(State newState){
        currentState = newState;
        robot.drivetrain.targetReached = false;
        timer.reset();
    }

}
