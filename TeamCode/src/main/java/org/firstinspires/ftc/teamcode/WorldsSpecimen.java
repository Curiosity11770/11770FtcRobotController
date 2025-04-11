package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Scoring;

@Config
@Autonomous(name="WorldsSpecimen", group="Linear OpMode")
public class WorldsSpecimen extends LinearOpMode {

    Robot robot;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime totalTime = new ElapsedTime();

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    //TODO Update states to reflect flow of robot actions
    enum State {
        DRIVE_TO_CHAMBER,
        DELIVER,
        STRAFE,
        DRIVE_TO_FLOOR_SAMPLE,
        RETRIEVE_SPECIMEN,
        PICKUP,
        DRIVE,
        IDLE
    }

    //Constants so these can be tuned in the dashboard
    public static double floorX = 48;
    public static double floorY = -27;
    public static double floorHeading = 0;

    public static double chamberX = 36;
    public static double chamberY = 5;
    public static double chamberHeading = 0;

    public static double pickupX = 4;
    public static double pickupY = -36;
    public static double pickupHeading = 0;


    // We define the current state we're on
    // Default to IDLE
    State currentState = State.DRIVE_TO_CHAMBER;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
    Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0);

    int specsScored;
    int samplesPushed;



    @Override
    public void runOpMode() {
        //calling constructor
        robot = new Robot(this);

        //calling init function
        robot.init();

        specsScored = 0;
        samplesPushed = 0;

        //TODO Pass starting pose to localizer
        //for Gobilda it looks like this
        //robot.drivetrain.localizer.odo.setPosition(startPose);
        //for sparkfun it looks like this
        robot.drivetrain.localizer.odo.setPosition(startPose);
        robot.drivetrain.targetReached = false;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        timer.reset();
        totalTime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case DRIVE_TO_CHAMBER:
                    if(timer.seconds() > 0.5 && specsScored == 0) {
                        robot.scoring.scoringMode = Scoring.ScoringMode.SCORING_HIGH_CHAMBER;
                    }
                    robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                    if(timer.seconds() > 1.0){
                        robot.drivetrain.profiledDriveToTarget(chamberX, chamberY+specsScored*3,chamberHeading);
                    }
                    if (robot.drivetrain.targetReached) {
                        switchState(State.DELIVER);
                    }
                    break;
                case DELIVER:
                    robot.scoring.clawServoPosition = Scoring.CLAW_OPEN;
                    robot.drivetrain.profiledDriveToTarget(24, chamberY, chamberHeading);
                    if(robot.drivetrain.targetReached && specsScored == 0){
                        switchState(State.STRAFE);
                        specsScored++;
                    }else if(robot.drivetrain.targetReached){
                        switchState(State.RETRIEVE_SPECIMEN);
                        specsScored++;
                    }
                    break;
                case STRAFE:
                    robot.scoring.scoringMode = Scoring.ScoringMode.PICKUP_WALL;
                    robot.lift.liftMode = Lift.LiftMode.GROUND;
                        robot.drivetrain.profiledDriveToTarget(24, floorY, floorHeading);
                    if(robot.drivetrain.targetReached){
                        switchState(State.DRIVE_TO_FLOOR_SAMPLE);
                        timer.reset();
                    }
                    break;
                case DRIVE_TO_FLOOR_SAMPLE:
                    if(timer.seconds() < 0.7 && samplesPushed == 0){
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 24, -27, AngleUnit.DEGREES, 0));
                    }else if(timer.seconds() < 1.3 && samplesPushed > 0){ //1.3
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 53, -24-(samplesPushed*8), AngleUnit.DEGREES, 0));
                        //53
                    }else if(timer.seconds() < 1.6){ //1.6
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 53, -29-(samplesPushed*8), AngleUnit.DEGREES, 0));
                    }else if(timer.seconds() < 1.9){ //1.9
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 53, -39-(samplesPushed*8), AngleUnit.DEGREES, 0));
                    }else if(timer.seconds() < 3.1){ //3.1
                        robot.drivetrain.setTargetPose(new Pose2D(DistanceUnit.INCH, 7.95, -39-(samplesPushed*8), AngleUnit.DEGREES, 0));
                    }else if(samplesPushed < 2){
                        timer.reset();
                        samplesPushed++;
                    }else{
                        switchState(State.RETRIEVE_SPECIMEN);
                        timer.reset();
                    }
                    break;
                case RETRIEVE_SPECIMEN:
                    robot.lift.liftMode = Lift.LiftMode.WALL_PICKUP;
                    robot.scoring.scoringMode = Scoring.ScoringMode.PICKUP_WALL;
                    robot.drivetrain.profiledDriveToTarget(pickupX, pickupY, pickupHeading);
                    if(robot.drivetrain.targetReached) {
                        switchState(State.PICKUP);
                        timer.reset();
                    }
                    break;
                case PICKUP:
                    robot.drivetrain.relativeDriveToTarget(-2, 0, 0, 0.08);
                   if(timer.seconds() > 0.8){
                       robot.scoring.clawServoPosition = Scoring.CLAW_CLOSED;
                   }
                    if(timer.seconds() > 1) {
                        switchState(State.DRIVE);
                        timer.reset();
                    }
                    break;
                case DRIVE:
                    robot.lift.liftMode = Lift.LiftMode.HIGH_CHAMBER;
                    if(timer.seconds() > 0.5) {
                        robot.scoring.scoringMode = Scoring.ScoringMode.SCORING_HIGH_CHAMBER;
                    }
                    robot.drivetrain.profiledDriveToTarget(10, chamberY+specsScored*3,chamberHeading);
                    if(robot.drivetrain.targetReached) {
                        switchState(State.DRIVE_TO_CHAMBER);
                        timer.reset();
                    }
                    break;
                case IDLE:
                    break;

            }

            // Anything outside of the switch statement will run independent of the currentState
            // We update robot continuously in the background, regardless of state
            robot.update();
            robot.scoring.update();
            robot.intake.update();
            robot.lift.update();

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
