package org.firstinspires.ftc.teamcode.old;




import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;
@Disabled
@TeleOp(name = "HeadingTeleOp", group = "TeleOp")
public class HeadingToAprilTags extends OpMode {
    DcMotorEx left_canon, right_canon,intake;

    final double TURN_GAIN   =  0.015  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value

    DcMotorEx outtake;
    double targetHeading = Math.toRadians(90), headingError = 0; // Radians
    boolean headingLock = false;
    private Follower follower;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry Dtelemetry = dashboard.getTelemetry();

    CameraLog camera = new CameraLog();
    public static Pose startingPose;
    private Supplier<PathChain> pathChainBlue, pathChainRed;
    private TelemetryManager telemetryM;
    public static double hp = 0.7;
    public static double hi = 0;
    public static double hd = 0.065 , hf = 0.03;

    static boolean canon_mode=false;
    public static double canon_velocity=1070, p = 1500, i = 1, d = 10, f = 0.0001, canon_angle = 25;

    PIDFController controller;

    public void init() {
        camera.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        controller = new PIDFController(new PIDFCoefficients(
                hp,
                hi,
                hd,
                hf));

        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        left_canon = hardwareMap.get(DcMotorEx.class, "shooter_l");
        right_canon = hardwareMap.get(DcMotorEx.class, "shooter_r");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
    }
    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    public void loop(){
        camera.desiredTag  = null;
        camera.telemetryAprilTag(telemetry, Dtelemetry);
        FtcDashboard.getInstance().startCameraStream(camera.visionPortal,300);

        left_canon.setVelocityPIDFCoefficients(p,i,d, f);
        right_canon.setVelocityPIDFCoefficients(p,i,d, f);
        follower.update();
        Dtelemetry.update();
        telemetryM.update();

        double error = targetHeading - follower.getHeading();  controller.setCoefficients(new PIDFCoefficients(
                hp,
                hi,
                hd,
                hf));
        controller.updateError(error);
        if (headingLock) {
            follower.setTeleOpDrive(-gamepad2.left_stick_y, -gamepad2.left_stick_x, controller.run(), false);
        }else if(camera.desiredTag != null) {
            headingError = camera.desiredTag.ftcPose.bearing;
            follower.setTeleOpDrive(-gamepad2.left_stick_y, -gamepad2.left_stick_x, Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN)/*-gamepad2.right_stick_x+(gamepad2.left_trigger-gamepad2.right_trigger)/2*/, false);
        }else
            follower.setTeleOpDrive(-gamepad2.left_stick_y, -gamepad2.left_stick_x,-gamepad2.right_stick_x+(gamepad2.left_trigger-gamepad2.right_trigger)/2, false);


        if (gamepad2.xWasPressed()){headingLock = false;}
        if (gamepad2.left_bumper){follower.setPose(new Pose (follower.getPose().getX(),follower.getPose().getY(), Math.toRadians(90)));targetHeading=Math.toRadians(90+canon_angle);headingLock = true;}
        if (gamepad2.right_bumper){follower.setPose(new Pose (follower.getPose().getX(),follower.getPose().getY(), Math.toRadians(90)));targetHeading=Math.toRadians(90-canon_angle);headingLock = true;}


        if(gamepad2.share){follower.setPose(new Pose (follower.getPose().getX(),follower.getPose().getY(), Math.toRadians(0)));}
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("canon", canon_mode);
        {
            //Basic control

            if (gamepad1.dpad_up && canon_velocity < 1440){canon_velocity+=1;}
            if (gamepad1.dpad_down&& canon_velocity > 950){canon_velocity-=1;}
            if (gamepad1.dpad_right){canon_velocity=1030;}
            if (gamepad1.dpad_up){canon_velocity=1140;}

            if(gamepad1.right_bumper){intake.setPower(-1);}
            else if (gamepad1.right_trigger>0.1){intake.setPower(gamepad1.right_trigger);}
            else{intake.setPower(0);}

            if(canon_mode){left_canon.setVelocity(canon_velocity, AngleUnit.DEGREES);right_canon.setVelocity(canon_velocity, AngleUnit.DEGREES);}
            else{left_canon.setPower(0);right_canon.setPower(0);}

            if(gamepad1.left_bumper &&( canon_velocity > canon_velocity - 20 )&&  (canon_velocity < canon_velocity + 20)){outtake.setPower(-1);}
            else if (gamepad1.left_trigger >0.1) {outtake.setPower(gamepad1.left_trigger);}
            else{outtake.setPower(0);}

            if (gamepad1.x) {outtake.setPower(1);}
        }
        telemetry.addData("position", follower.getPose());
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.addData("canon", canon_mode);
        telemetry.addData("Var_canon_velocity", canon_velocity);
        telemetry.addData("left_canon_velocity_in_degrees", left_canon.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("right_canon_velocity_in_degrees", right_canon.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("intake_velocity_in_degrees", intake.getVelocity(AngleUnit.DEGREES));
        telemetry.update();
        telemetryM.addData("Var_canon_velocity", canon_velocity);
        telemetryM.addData("left_canon_velocity_in_degrees", left_canon.getVelocity(AngleUnit.DEGREES));
        telemetryM.addData("right_canon_velocity_in_degrees", right_canon.getVelocity(AngleUnit.DEGREES));
        telemetryM.addData("intake_velocity_in_degrees", intake.getVelocity(AngleUnit.DEGREES));
        telemetryM.update();

//    if( (gamepad2.right_stick_x != 0)  (gamepad2.right_stick_y != 0)  (gamepad2.left_stick_x != 0) || (gamepad2.left_stick_y != 0)){
//            intake.setPower(-1);
//        }

    }

}