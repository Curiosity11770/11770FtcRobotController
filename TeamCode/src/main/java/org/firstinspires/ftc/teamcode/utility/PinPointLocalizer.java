package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utility.GoBildaPinpointDriver;

import java.util.Locale;

public class PinPointLocalizer {

    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    public Pose2D pos;

    public PinPointLocalizer(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
         /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
        */
        odo.setOffsets(60.0, 80.0); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

        myOpMode.telemetry.addData("Status", "Initialized");
        myOpMode.telemetry.addData("X offset", odo.getXOffset(DistanceUnit.INCH));
        myOpMode.telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.INCH));
        myOpMode.telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        myOpMode.telemetry.addData("Device SCalar", odo.getYawScalar());
    }

    public void update(){
        odo.update();
         /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
        pos = odo.getPosition();

        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        //odo.updateDashboard();
        myOpMode.telemetry.addData("Position", data);
    }

    public double getX(){
        return pos.getX(DistanceUnit.INCH);
    }

    public double getY(){
        return pos.getY(DistanceUnit.INCH);
    }

    public double getHeading(){
        return pos.getHeading(AngleUnit.DEGREES);
    }

}
