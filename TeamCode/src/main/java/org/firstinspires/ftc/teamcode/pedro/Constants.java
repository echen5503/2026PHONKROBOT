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
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.9999138539231669);
        c.yPodOffset.set(-5.90617262472318);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // UNDER SCRUTINY
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4844140619746826);
                Controller secondaryTranslationalForward = Controller.proportional(0.17897805211752024);
                Controller primaryTranslationalLateral = Controller.proportional(1.4453524876775896);
                Controller secondaryTranslationalLateral = Controller.proportional(0.5340191236671141);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014560474755109451));
                c.brake.set(Controller.proportionalFeedforward(0.012376403541843033));

                c.headingFeedback.set(Controller.proportional(8.995367258031758));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05296845882403956, 0.007582410698505151));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09329031896256419, 0.04487436317106063));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0019579540531474058, 0.002293028719160252));

                c.maxAchievableForwardVelocity.set(69.29183080589395);
                c.maxAchievableStrafeVelocity.set(50.27447198778312);
                c.naturalForwardDeceleration.set(32.761009642728474);
                c.naturalStrafeDeceleration.set(77.00490413398312);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}