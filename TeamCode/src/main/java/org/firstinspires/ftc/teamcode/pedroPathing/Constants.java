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
                .forwardZeroPowerAcceleration(-32.33603205192884)
                .lateralZeroPowerAcceleration(-73)
                .translationalPIDFCoefficients(new PIDFCoefficients(0.083,0,0.001,0.05))
                .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0,0.00000000025,1,0.07))
                .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.001,0.04));

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
                .xVelocity(73.337903120386315)
                .yVelocity(57.66839599609376);
        //using shooter l
        public static PinpointConstants localizerConstants = new PinpointConstants()
                .forwardPodY(-115)
                .strafePodX(-155)
                .distanceUnit(DistanceUnit.MM)
                .hardwareMapName("pinpoint")
                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        public static PathConstraints pathConstraints = new PathConstraints
                (
                        0.99,
                        1,
                        1,
                        0.05,
                        150,
                        0.85,
                        20,
                        0.7
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