package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Robot {
    private LinearOpMode myOpMode = null;

    Drivetrain drivetrain;

    Intake intake;

    Power power;

    public Robot(LinearOpMode opmode){
        myOpMode = opmode;
    }
    public void init(){
        drivetrain = new Drivetrain(myOpMode);
        intake = new Intake(myOpMode);
        power = new Power(myOpMode);

        drivetrain.init();
        intake.init();
        power.init();
    }

    public void update(){
        drivetrain.update();
        intake.update();
        power.update();
    }
    public void teleOp() {
        drivetrain.teleOp();
        intake.teleOp();
        power.teleOp();
    }
}
