package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous (name="RedRightAuto", group = "Concept")
public class RedRightAuto extends LinearOpMode {
    private Robot robot;
    private ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() {
        robot = new Robot(this);
        robot.init();

        robot.drivetrain.localizer.setCoordinates(12, 66, Math.PI/2);

        while (!isStarted()) {
            robot.camera.scanAprilTag(5);
            telemetry.addData("Position: ", robot.camera.returnSelection());
            telemetry.update();
        }

        robot.camera.visionPortalFront.stopStreaming();
        // robot.camera.stopColorProcessor();

        waitForStart();

        if(robot.camera.returnSelection() == SimpleVisionProcessor.Selected.MIDDLE){
            robot.drivetrain.driveStraightPID(35, 3);
            runtime.reset();
            while(runtime.seconds() < 3){
                robot.intake.outtake(0.8);
            }
            robot.intake.intakeLeft.setPower(0);
            robot.intake.intakeRight.setPower(0);
            robot.drivetrain.encoderTurn(3400, 3);
            robot.driveToAprilTag(2, 4);
            runtime.reset();
            while(runtime.seconds() < 2) {
                robot.lift.liftLeft.setPower(0.5);
                robot.lift.liftRight.setPower(0.5);
            }

            robot.lift.liftLeft.setPower(0);
            robot.lift.liftRight.setPower(0);

            robot.scoring.leftArmServo.setPosition(robot.scoring.ARM_UP_LEFT);
            robot.scoring.rightArmServo.setPosition(robot.scoring.ARM_UP_RIGHT);
            sleep(3000);
            robot.scoring.leftGateServo.setPosition(robot.scoring.ARM_UP_LEFT);
            robot.scoring.rightGateServo.setPosition(robot.scoring.ARM_UP_RIGHT);
            sleep(3000);
            robot.drivetrain.driveSidePID(10,5);
            //robot.lift.liftToPositionPIDClass(100);
            //robot.scoring.(100);

            // robot.l
        } else if(robot.camera.returnSelection() == SimpleVisionProcessor.Selected.LEFT) {

        }

        //try drive to pose
        //try drive to april tag
    }

}