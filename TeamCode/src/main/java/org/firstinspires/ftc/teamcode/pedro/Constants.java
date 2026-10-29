package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFrontDrive");
        c.frontRightName.set("rightFrontDrive");
        c.backLeftName.set("leftBackDrive");
        c.backRightName.set("rightBackDrive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        // c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(4.294028995543953);
        c.yPodOffset.set(-1.1472092275544414);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.32643984177075935);
                Controller secondaryTranslationalForward = Controller.proportional(0.12061079890107645);
                Controller primaryTranslationalLateral = Controller.proportional(0.5411054182778577);
                Controller secondaryTranslationalLateral = Controller.proportional(0.19992399345060416);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013790010903365712));
                c.brake.set(Controller.proportionalFeedforward(0.011721509267860855));

                c.headingFeedback.set(Controller.proportional(4.4848503368496395));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04625272705502396, 0.00504835273079934));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07830304904074425, 0.062406113906291284));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015266460178899911, 0.0016204120182627814));

                c.maxAchievableForwardVelocity.set(76.02965888023712);
                c.maxAchievableStrafeVelocity.set(62.693606666801465);
                c.naturalForwardDeceleration.set(37.34133441100242);
                c.naturalStrafeDeceleration.set(71.4422381702293);
            }
    );
}