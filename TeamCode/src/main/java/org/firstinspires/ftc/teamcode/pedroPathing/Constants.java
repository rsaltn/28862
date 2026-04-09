package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
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
@Configurable
public class Constants
{
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.43)
            .forwardZeroPowerAcceleration(-28.33603205192884)
            .lateralZeroPowerAcceleration(-65)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0.15))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0,0.00000000025,1,0.07))
            .headingPIDFCoefficients(new PIDFCoefficients(0.65,0,0.01,0.03));

    //    -7152.396568518804
    //    -217.37893357833778
    public static MecanumConstants driveConstants = new MecanumConstants()

            .maxPower(0.95)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .xVelocity(80.337903120386315)
            .yVelocity(55.66839599609376);
    //using shooter l
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(20)
            .strafePodX(-90)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints
            (
                    0.995,
                    0.5,
                    0.5,
                    0.03,
                    100,
                    0.4,
                    20,
                    0.1
            );
    public static Follower createFollower(HardwareMap hardwareMap)
    {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

}