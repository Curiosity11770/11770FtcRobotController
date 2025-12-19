package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="FieldOrientedTeleOp", group="Linear OpMode")
public class FieldOrientedTeleop extends LinearOpMode {

    //declaring instance of robot hardware
    Robot robot = new Robot(this);
    private Follower follower;

    enum DriveMode{
        ROBOT_CENTRIC,
        RED_FIELD_CENTRIC,
        BLUE_FIELD_CENTRIC
    }
    DriveMode driveMode = DriveMode.ROBOT_CENTRIC;

    @Override
    public void runOpMode() {
        //calling constructor
        // robot = new RobotHardware(this);
        //calling init function
        robot.init();
        // Wait for the game to start (driver presses START)
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();
        follower.startTeleopDrive(true);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                driveMode = DriveMode.ROBOT_CENTRIC;
            } else if (gamepad1.dpad_left) {
                driveMode = DriveMode.BLUE_FIELD_CENTRIC;
            } else if (gamepad1.dpad_right) {
                driveMode = DriveMode.RED_FIELD_CENTRIC;
            }

            follower.update();
            if (driveMode == DriveMode.ROBOT_CENTRIC) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                );
            } else if (driveMode == DriveMode.BLUE_FIELD_CENTRIC) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false
                );
            } else if (driveMode == DriveMode.RED_FIELD_CENTRIC) {
                follower.setTeleOpDrive(
                        gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false
                );
            }
            telemetry.addData("Drive Mode: ", driveMode);
            telemetry.update();
        }
    }
}