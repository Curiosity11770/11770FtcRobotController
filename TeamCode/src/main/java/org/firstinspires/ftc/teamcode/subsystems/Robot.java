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

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();


    public Robot(LinearOpMode opmode){
        myOpMode = opmode;
    }
    public void init(){
        drivetrain = new Drivetrain(myOpMode);
        intake = new Intake(myOpMode);
        power = new Power(myOpMode);
        shooter = new Shooter (myOpMode);
        spindexer = new Spindexer(myOpMode);

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

       /* if (myOpMode.gamepad2.y){
            runningActions.add(new SequentialAction(
                    shooter.shooterAction(shooter.TICKS_PER_SECOND),
                    spindexer.spindexerAction(0.6),
                    shooter.transferAction(shooter.TRANSFER_SPEED),
                    shooter.linkageAction(shooter.LINKAGE_UP)
                    ));
        }*/
    }

}

