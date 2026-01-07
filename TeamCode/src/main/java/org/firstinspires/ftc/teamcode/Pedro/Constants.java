package org.firstinspires.ftc.teamcode.Pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0.0, 0.01, 0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0.0, 0.075, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.021, 0.0, 0.00025, 0.6, 0.07))
            .forwardZeroPowerAcceleration(-34.99)
            .lateralZeroPowerAcceleration(-67.109)
            .centripetalScaling(0.0025)
            .mass(10);

    public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 1.0, 1.0);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .useBrakeModeInTeleOp(true)
            .maxPower(1.0)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(69.83)
            .yVelocity(54.61);

    public static PinpointConstants localizerPinpointConstants = new PinpointConstants()
            .forwardPodY(4.125)
            .strafePodX(6.4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerPinpointConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
