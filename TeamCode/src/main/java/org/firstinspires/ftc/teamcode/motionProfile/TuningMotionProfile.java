package org.firstinspires.ftc.teamcode.motionProfile;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name="TuningMotionProfile", group="Linear OpMode")
@Config
public class TuningMotionProfile extends LinearOpMode {

    Robot robot;

    ElapsedTime timer = new ElapsedTime();

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    //TODO Update states to reflect flow of robot actions
    enum State {
        DRIVE_TO_TARGET,
        IDLE_ONE,
        DRIVE_TO_START,
        IDLE_TWO
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.DRIVE_TO_TARGET;

    // Define our start pose
    Pose2D startPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES,0);

    // Define our target
    public static double targetX = 24;
    public static double targetY = 0;
    public static double targetHeading = 0;

    @Override
    public void runOpMode() {
        //calling constructor
        robot = new Robot(this);

        //calling init function
        robot.init();

        //TODO Pass starting pose to localizer
        //for Gobilda it looks like this
        //robot.drivetrain.localizer.odo.setPosition(startPose);
        //for sparkfun it looks like this
        //robot.drivetrain.localizer.myOtos.setPosition(startPose);

        //Set the drivetrain's first target
        //robot.drivetrain.setTargetPose(startPose);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState){
                case DRIVE_TO_TARGET:
                    robot.drivetrain.profiledDriveToTarget(targetX, targetY, targetHeading);
                    if(robot.drivetrain.targetReached){
                        setState(State.IDLE_ONE);
                    }
                    break;
                case IDLE_ONE:
                    robot.drivetrain.profiledDriveToTarget(targetX, targetY, targetHeading);
                    if(timer.seconds() > 0.1){
                        setState(State.DRIVE_TO_START);
                    }
                    break;
                case DRIVE_TO_START:
                    robot.drivetrain.profiledDriveToTarget(0, 0, 0);
                    if(robot.drivetrain.targetReached){
                        setState(State.IDLE_TWO);
                    }
                    break;
                case IDLE_TWO:
                    robot.drivetrain.profiledDriveToTarget(0, 0, 0);
                    if(timer.seconds() > 0.1){
                        setState(State.DRIVE_TO_TARGET);
                    }
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            robot.update();

            telemetry.addData("state", currentState);
            telemetry.update();
        }
    }

    void setState(State state){
        currentState = state;
        timer.reset();
    }

}

