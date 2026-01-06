package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(4.5)
            .forwardZeroPowerAcceleration(-52.425830770605764)
            .lateralZeroPowerAcceleration(-70.269834750347598)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08,0.0,0.0067,0.033))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.03, 0.028))
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99,

            90,
            0.92,
            0.9);

    // ✅ Drivetrain constants (aligned with TeleOp motor names/directions)
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(38.54861877529901)
            .yVelocity(26.826)
            ;

    // ✅ Pinpoint localizer constants
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("frontLeft")
            .strafeEncoder_HardwareMapName("backLeft")
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .forwardTicksToInches(0.00303)
            .strafeTicksToInches(-0.002945)
            //.forwardPodY(5.2)    // adjust based on your robot’s actual offset
            //.strafePodX(1.7)   // adjust based on your robot’s actual offset
            .forwardPodY(6)    // adjust based on your robot’s actual offset
            .strafePodX(-1)   // adjust based on your robot’s actual offset
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
                );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)     // ✅ attach drivetrain
                .twoWheelLocalizer(localizerConstants) // ✅ attach localizer
                .build();
    }
}
