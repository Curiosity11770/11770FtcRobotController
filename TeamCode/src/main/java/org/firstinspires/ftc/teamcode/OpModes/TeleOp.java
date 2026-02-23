package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Linear Opmode")

public class TeleOp extends LinearOpMode {
    Robot robot = new Robot(this);
    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode() {
        robot.init();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("loopTime", runtime.milliseconds());
            runtime.reset();
            robot.teleOp();
            robot.update();

            telemetry.update();
        }
    }

}
