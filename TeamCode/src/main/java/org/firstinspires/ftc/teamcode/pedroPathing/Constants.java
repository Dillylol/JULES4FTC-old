package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
        public static MecanumConstants driveConstants = new MecanumConstants()
                        .maxPower(1)
                        .rightFrontMotorName("rf")
                        .rightRearMotorName("rr")
                        .leftRearMotorName("lr")
                        .leftFrontMotorName("lf")
                        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                        .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .xVelocity(60.310722462487355)
                        .yVelocity(39.7916370492672);

        public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
                        .forwardTicksToInches(.001989436789)
                        .strafeTicksToInches(.001989436789)
                        .turnTicksToInches(.001989436789)
                        .leftPodY(2.5)
                        .rightPodY(-2.5)
                        .strafePodX(-2.5)
                        .leftEncoder_HardwareMapName("lf")
                        .rightEncoder_HardwareMapName("lr")
                        .strafeEncoder_HardwareMapName("rf")
                        .leftEncoderDirection(Encoder.REVERSE)
                        .rightEncoderDirection(Encoder.REVERSE)
                        .strafeEncoderDirection(Encoder.FORWARD)
                        .IMU_HardwareMapName("imu")
                        .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                        RevHubOrientationOnRobot.UsbFacingDirection.UP));

        public static FollowerConstants followerConstants = new FollowerConstants()
                        .mass(11.158) //change
                        .forwardZeroPowerAcceleration(-43.0295022127659405)
                        .lateralZeroPowerAcceleration(-91.256410323735925)
                        .useSecondaryTranslationalPIDF(false)
                        .useSecondaryHeadingPIDF(false)
                        .useSecondaryDrivePIDF(false)
                        .translationalPIDFCoefficients(new PIDFCoefficients(0.0795, 0.0000, 0.0015, 0.0212)) //tune
                        .headingPIDFCoefficients(new PIDFCoefficients(0.75, 0, 0.0015, 0.0315)) //tune
                        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0.0, 0.0006, 0.6, 0.0212))
                        .centripetalScaling(0.0005);
        public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, .975, 1);

        public static Follower createFollower(HardwareMap hardwareMap) {
                return new FollowerBuilder(followerConstants, hardwareMap)
                                .threeWheelIMULocalizer(localizerConstants)
                                .pathConstraints(pathConstraints)
                                .mecanumDrivetrain(driveConstants)
                                .build();

        }
}
