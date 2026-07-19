package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

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


    GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .leftFrontMotorName("fl")
            .rightFrontMotorName("fr")
            .leftRearMotorName("bl")
            .rightRearMotorName("br")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(64.38)
            .yVelocity(53.35);



    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.25)
            .strafePodX(-7.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.5)
            .forwardZeroPowerAcceleration(-38.51)
            .lateralZeroPowerAcceleration(-42.87)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.04,0.000,0.001, 0.045))
            .headingPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.0005, 0.3))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.00001, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0003,0,0.00007,0.01,0.25))
            .centripetalScaling(0.0002);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}