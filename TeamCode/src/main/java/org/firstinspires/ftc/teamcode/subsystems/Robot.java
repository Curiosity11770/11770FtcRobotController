package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Action;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    private LinearOpMode myOpMode = null;

    public Drivetrain drivetrain;

    Intake intake;

    Power power;

    Shooter shooter;

    Spindexer spindexer;

    Vision vision;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();


    public Robot(LinearOpMode opmode){
        myOpMode = opmode;
    }
    public void init(){
        vision = new Vision(myOpMode);
        drivetrain = new Drivetrain(myOpMode, vision);
        intake = new Intake(myOpMode);
        power = new Power(myOpMode);
        shooter = new Shooter (myOpMode, vision);
        spindexer = new Spindexer(myOpMode, intake, shooter);


        vision.init();
        drivetrain.init();
        intake.init();
        power.init();
        shooter.init();
        spindexer.init();
    }

    public void update(){
        drivetrain.update();
        intake.update();
        power.update();
        shooter.update();
        spindexer.update();
    }
    public void teleOp() {
        drivetrain.teleOp();
        intake.teleOp();
        power.teleOp();
        shooter.teleOp();
        spindexer.teleOp();

        TelemetryPacket packet = new TelemetryPacket();

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }

}

