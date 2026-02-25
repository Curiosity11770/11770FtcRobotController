package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestTeleOp", group = "Linear Opmode")

public class TestTeleop extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain(this);
    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode() {
        drivetrain.init();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("loopTime", runtime.milliseconds());
            runtime.reset();
            drivetrain.teleOpExtra();
            telemetry.update();
        }
    }

}
