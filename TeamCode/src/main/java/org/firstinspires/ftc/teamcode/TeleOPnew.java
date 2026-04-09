package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;


@Configurable
@TeleOp(name = "TeleOpnew frr", group = "TeleOp")
public class TeleOPnew extends OpMode {

    Follower follower;
    GoBildaPinpointDriver s_pinpoint;


    CRServo locker;
    DcMotorEx intake, Heading;

    TelemetryManager telemetryP;


    public static Pose startingPose;

    Shooter shooter = new Shooter();

    public double[] data;
    public static double velocity = 0, angle = 0, delayT = 0.15, constantT = -2.58, sp = 0.07,si = 0.0001,sd = 0.0006;




    public static boolean autoAdjacement = false, lockerMode = true;

    public static double targetHeading = 0;


    public static double tagHoldSeconds = 0.15;

    private double lastSeenTime = -999;
    public static double TURN_GAIN = 0.015;
    public static double MAX_AUTO_TURN = 0.15 ;

    Timer serv = new Timer();
    Timer move = new Timer();
    private boolean autoAim = true, delay = false, breakM = false, llvresult = true;

    // Переменные для отслеживания состояния выстрела
    private boolean isShootingInProgress = false; // чтобы предотвратить многократные выстрелы
    private int ballsShot = 0; // Счётчик выстреленных мячей

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        data = shooter.createCanonData(velocity, angle);
        Constants.driveConstants.maxPower(1);
        locker.setPower(0.4);
    }

    @Override
    public void init() {
        s_pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "s_pinpoint");

        s_pinpoint.setPosition(new Pose2D(DistanceUnit.CM,0,0,AngleUnit.DEGREES,0));

        follower = Constants.createFollower(hardwareMap);

        Constants.driveConstants.maxPower(1);

        Heading = hardwareMap.get(DcMotorEx.class, "s_heading");

        locker = hardwareMap.get(CRServo.class,"locker");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        shooter.init(hardwareMap, true);

        telemetryP = PanelsTelemetry.INSTANCE.getTelemetry();


        Heading.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);


        s_pinpoint.update();

        telemetry.clearAll();

        follower.update();

        intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);


//
//        if(llvresult)
//        {
//            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//            if (fiducials != null && !fiducials.isEmpty())
//            {
//                double x1 = fiducials.get(0).getTargetCorners().get(0).get(0);
//                double y1 = fiducials.get(0).getTargetCorners().get(0).get(1);
//                double x2 = fiducials.get(0).getTargetCorners().get(1).get(0);
//                double y2 = fiducials.get(0).getTargetCorners().get(1).get(1);
//                double x3 = fiducials.get(0).getTargetCorners().get(2).get(0);
//                double y3 = fiducials.get(0).getTargetCorners().get(2).get(1);
//                double x4 = fiducials.get(0).getTargetCorners().get(3).get(0);
//                double y4 = fiducials.get(0).getTargetCorners().get(3).get(1);
//                double corner1 = Math.pow(Math.pow(x1-x4,2)+Math.pow(y1-y4,2),0.5);
//                double corner2 = Math.pow(Math.pow(x2-x3,2)+Math.pow(y2-y3,2),0.5);
//                telemetryP.addData("Left Corner = ", corner1);
//                telemetryP.addData("Right Corner = ", corner2);
//                telemetryP.addData("Average Corner = ", (corner1+corner2)/2);
//                double x = (corner1+corner2)/2;
//                double y = -0.00407296 * Math.pow(x,3) + 1.17902 * Math.pow(x,2) - 119.1841 * x + 7583.71974;
//                //122 - 3200,0, 110 - 3300,20, 100 - 3400,30, 90 - 3450, 35, 80 - 3500,40, 70 - 3600,50, 60 - 3800,65, 50 - 4100,80, 40 - 4400,100, 35 - 4700,125
//                double z = -0.000369038* Math.pow(x,3) + 0.0964091 * Math.pow(x,2) - 9.01097 * x + 335.01112;
//
//                telemetryP.addData("Turret power = ", y);
//                telemetryP.addData("Turret angle = ", z);
//                if(gamepad1.rightBumperWasPressed())
//                {
//                    autoAdjacement = !autoAdjacement;
//                }
//                if(autoAdjacement)
//                {
//                    autoAim = true;
//                    data = shooter.createCanonData(y, z);
//                }
//            }
//
//        }


        if(gamepad1.dpadDownWasPressed()) {lockerMode = !lockerMode;}
        if(lockerMode){locker.setPower(0.4);}
        else{locker.setPower(0.05);gamepad1.rumble(1,1,100);}

        //Heading.setPower((gamepad1.left_bumper?-0.3:gamepad1.right_bumper?0.3:0));

        Heading.setPower(Math.abs(targetHeading-s_pinpoint.getHeading(AngleUnit.DEGREES))>0.5?targetHeading-s_pinpoint.getHeading(AngleUnit.DEGREES)/45:0);

        targetHeading += (gamepad1.left_bumper ? -0.1 : gamepad1.right_bumper ? 0.1 : 0);


        if(gamepad1.touchpad){s_pinpoint.resetPosAndIMU();}


        if (gamepad1.square) {
            autoAdjacement = false;
            data = shooter.createCanonData(shooter.getTargetRpm()+(gamepad1.dpad_left?-1:1), 0);
//            servo.setPower(-0.7);
            autoAim = true;
        }
        if (gamepad1.triangleWasPressed()) {
            autoAdjacement = false;
            data = shooter.createCanonData(980, 60);
            autoAim = true;
        }
        if (gamepad1.circleWasPressed()) {
            autoAdjacement = false;
            data = shooter.createCanonData(1200,140);
            autoAim = true;
        }
        if (gamepad1.crossWasPressed()) {
            autoAdjacement = false;
            data = shooter.createCanonData(0, 0);
            autoAim = false;
        }
        shooter.shootON(data);

        if(gamepad1.share){follower.setPose(new Pose (follower.getPose().getX(),follower.getPose().getY(), Math.toRadians(0)));telemetry.addData("Used", getRuntime());}
        telemetry.addData("Shooter target RPM", shooter.getTargetRpm());
        telemetry.addData("Shooter meas RPM", shooter.getMeasuredRpm());
        telemetry.addData("Shooter right", shooter.right_canon.getVelocity());
        telemetry.addData("Shooter left", shooter.left_canon.getVelocity());
        telemetry.addData("Shooter right pos", shooter.right_canon.getCurrentPosition());
        telemetry.addData("Shooter left pos", shooter.left_canon.getCurrentPosition());
        telemetry.addData("Shooter power", shooter.getLastPower());
        telemetry.addData("VelocityX", follower.drivetrain.xVelocity());
        telemetry.addData("Loop", getRuntime());
        telemetry.addData("LSY", gamepad1.left_stick_y);
        telemetry.addData("share", gamepad1.share);
        telemetry.addData("s_pinpoint in degrees", s_pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("s_heading target degree", targetHeading);
        telemetry.update();
        telemetryP.update();

    }

}
