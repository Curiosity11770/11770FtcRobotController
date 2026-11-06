package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.RobotHardware;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Linear OpMode")
public class TeleOp extends LinearOpMode {

    //declaring instance of robot hardware
    RobotHardware robot;
    private Follower follower;
    public static Pose startingPose;
    public double speedMultiplier = 0.75;
    public ElapsedTime loopTime;
    public int pipeline;

    double kP = 0.01;
    double kI = 0;
    double kD = 0;

    public static double targetX = 58;
    public static double targetY = 51;

    Controller headingController;
    double headingOutput;

    enum DriveMode {
        ROBOT_CENTRIC,
        RED_FIELD_CENTRIC,
        BLUE_FIELD_CENTRIC
    }

    DriveMode driveMode = DriveMode.ROBOT_CENTRIC;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(this);
        robot.init();
        follower = Constants.create(hardwareMap);
        headingController = Controller.pid(kP, kI, kD);
        headingOutput = 0;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        loopTime = new ElapsedTime();

        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();

        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("loopTime", loopTime.milliseconds());
            loopTime.reset();

            follower.update();
            robot.teleOp();

            double turnPower = 0;
            if (gamepad1.left_trigger > 0.5) {
                double angle = Math.atan2(targetY - follower.pose().y(), targetX - follower.pose().x());
                angle = (angle + Math.toRadians(180)) % (Math.PI * 2);
                telemetry.addData("Angle target: ", angle);
                telemetry.addData("Robot heading radians: ", follower.pose().heading());
                turnPower = headingController.calculate(angle, angle - follower.pose().heading());
            }
            headingOutput = -gamepad1.right_stick_x + turnPower * 100;

            if (gamepad1.dpad_left) {
                driveMode = DriveMode.ROBOT_CENTRIC;
            } else if (gamepad1.dpad_right) {
                driveMode = DriveMode.BLUE_FIELD_CENTRIC;
            }

            if (driveMode == DriveMode.ROBOT_CENTRIC) {
                ManualDrive.driveOrHold(
                        follower,
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        headingOutput
                );
                follower.update();
            } else if (driveMode == DriveMode.BLUE_FIELD_CENTRIC) {
                DrivePowers powers = ManualDrive.fieldCentric(
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        headingOutput,
                        follower.pose().heading()
                );
                ManualDrive.driveOrHold(follower, powers);
                follower.update();
            } else if (driveMode == DriveMode.RED_FIELD_CENTRIC) {
            }

            /*if (gamepad1.right_bumper) {
                speedMultiplier = 1;
            } else if (gamepad1.left_bumper) {
                speedMultiplier = 0.25;
            } else {
                speedMultiplier = 0.75;
            }*/

            Pose robotPose = follower.pose();
            telemetry.addData("Drive Mode: ", driveMode);
            telemetry.addData("Turn power: ", turnPower);
            telemetry.addData("pipeline", pipeline);
            telemetry.addData("Robot X", robotPose.x());
            telemetry.addData("Robot Y", robotPose.y());
            telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
            telemetry.addData("Nectar RPM:", robot.launcher.NECTAR_RPM);
            telemetry.addData("Pollen RPM:", robot.launcher.POLLEN_RPM);
            telemetry.addData("Nectar Gate Pos", robot.transfer.nectarGate.getPosition());
            telemetry.addData("Pollen Gate Pos", robot.transfer.pollenGate.getPosition());
            telemetry.addData("Nectar Launcher Vel", robot.launcher.nectarLauncher.getVelocity() * 60 / Launcher.TICKS_PER_REVOLUTION);
            telemetry.addData("Pollen Launcher Vel", robot.launcher.pollenLauncher.getVelocity() * 60 / Launcher.TICKS_PER_REVOLUTION);
            telemetry.update();
        }
    }
}