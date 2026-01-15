package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

public class Vision {
    private LinearOpMode myOpMode = null;
    public Limelight3A limelight = null;

    public LLResult result;

    public List<LLResultTypes.FiducialResult> fiducials;

    public Vision (LinearOpMode opmode) {
        myOpMode = opmode;
    }
    public void init (){
        limelight = myOpMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(1);
        limelight.getLatestResult();

        result = limelight.getLatestResult();

        fiducials = result.getFiducialResults();
        limelight.start();
    }

    public void teleOp() {
    }


}
