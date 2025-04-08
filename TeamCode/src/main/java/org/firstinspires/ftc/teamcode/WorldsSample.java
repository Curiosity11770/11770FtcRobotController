package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

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
        DRIVE_TO_SUBMERSIBLE,
        RETRIEVE_SUBMERSIBLE_SAMPLE,
        DRIVE_TO_BASKET_FROM_SUBMERSIBLE,
        PARK,
        IDLE
    }

    //Constants so these can be tuned in the dashboard
    public static double basketX = 8;
    public static double basketY = 10;
    public static double basketHeading = -45;

    public static double floorSampleX = 24;
    public static double floorSampleY = 9;
    public static double floorSampleHeading = 0;

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
                    robot.drivetrain.profiledDriveToTarget(basketX, basketY,basketHeading);
                    //put condition for switch at the end, condition can be based on time or completion of a task
                    if(robot.drivetrain.targetReached || timer.seconds() > 1.5){
                        switchState(State.DELIVER_SAMPLE);
                    }
                    break;
                case DELIVER_SAMPLE:
                    robot.drivetrain.profiledDriveToTarget(basketX, basketY,basketHeading);
                    if(timer.seconds() > 1.5 && samplesScored < 3){
                        switchState(State.DRIVE_TO_FLOOR_SAMPLE);
                        samplesScored++;
                    }else if(timer.seconds() >1.5){
                        switchState(State.DRIVE_TO_SUBMERSIBLE);
                        samplesScored++;
                    }
                    break;
                case DRIVE_TO_FLOOR_SAMPLE:
                    telemetry.addData("expression", floorSampleY+samplesScored*6.0);
                    robot.drivetrain.profiledDriveToTarget(floorSampleX, floorSampleY + (samplesScored-1)*6.0,floorSampleHeading+(samplesScored-1)*15.0);
                    if(timer.seconds() > 1.5){
                        switchState(State.RETRIEVE_FLOOR_SAMPLE);
                    }
                    break;
                case RETRIEVE_FLOOR_SAMPLE:
                    if(timer.seconds() > 1.5){
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
