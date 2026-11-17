package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Tuned values copied from the AutoTune project (Quickstart/tuningTest) on 2026-11-17.
// "old tuning" comments are the values from the earlier tuning run.
public class Constants {




    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("RF");
        c.frontRightName.set("RB");
        c.backLeftName.set("LB");
        c.backRightName.set("LF");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });


    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.5142956530953953);  // old tuning: -1.0326459839588076
        c.yPodOffset.set(-6.924308716781496);  // old tuning: 0.34603302873025726
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.7180905858274919);  // old tuning: 0.23198611213985942
                Controller secondaryTranslationalForward = Controller.proportional(0.2653152837294193);  // old tuning: 0.08571266965259705
                Controller primaryTranslationalLateral = Controller.proportional(3.0);  // old tuning: 0.30691330160217595 | autotune said 4.684002701082409, lowered to stop side-to-side wobble
                Controller secondaryTranslationalLateral = Controller.proportional(1.0);  // old tuning: 0.11339626406754752 | autotune said 1.7306138391926933, lowered to stop side-to-side wobble
                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018433279618911984));  // old tuning: 0.01139351744417845
                c.brake.set(Controller.proportionalFeedforward(0.015668287676075185));  // old tuning: 0.009684489827551683
                c.headingFeedback.set(Controller.proportional(6.047358833333193));  // old tuning: 3.9427506057122628 | autotune said 7.647358833333193, lowered by hand
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05217022046993515, 0.007883074960817471));  // old tuning: 0.04895982279838658, 0.006568453088979415
                c.linearBrakeCoefficients.set(Matrix.diag(0.06041105904647288, 0.03517110011658868));  // old tuning: 0.0952642831358386, -0.05946699916014508
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015524062133022063, 0.0022433293926270546));  // old tuning: 0.001094743795020379, 0.004633197539731494
                c.maxAchievableForwardVelocity.set(58.33039144069742);  // old tuning: 78.35572886083301
                c.maxAchievableStrafeVelocity.set(23.27063120616697);  // old tuning: 64.59641490755081
                c.naturalForwardDeceleration.set(63.18809338512128);  // old tuning: 166.37404796160075
                c.naturalStrafeDeceleration.set(67.04420622347911);  // old tuning: 61.16263402159703
            }

    );


    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    // ===== Helpers for things Pedro 2.x had built in =====

    /**
     * Replacement for the old follower.setMaxPower(power) during path following.
     * Pedro 3.x limits path SPEED (inches/sec) instead of motor power, so this caps the
     * speed at that fraction of the robot's tuned top speed. 1.0 = no limit.
     */
    public static void setMaxPathPower(double power) {
        foresightConfig.maxPathSpeed.set(power >= 1.0
                ? ForesightConfig.Constraint.NONE
                : power * foresightConfig.maxAchievableForwardVelocity.get());
    }

    /** Replacement for the old Pose.mirror(): flips a blue pose to the red side (field is 144 in). */
    public static Pose mirror(Pose pose) {
        return new Pose(144 - pose.x(), pose.y(), Angle.normalize(Math.PI - pose.heading()));
    }
}
