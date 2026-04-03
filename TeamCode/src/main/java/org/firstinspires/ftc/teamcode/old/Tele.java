package org.firstinspires.ftc.teamcode.old;




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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;
@Disabled

@TeleOp(name = "TeleOpCentricField", group = "TeleOp")
public class Tele extends OpMode {
    DcMotorEx left_canon, right_canon,intake, outtake;

    double targetHeading = Math.toRadians(90); // Radians
    boolean headingLock = false;
    private Follower follower;

    public static Pose startingPose;
    private Supplier<PathChain> pathChainBlue, pathChainRed;
    private TelemetryManager telemetryM;
    public static double hp = 0.7;
    public static double hi = 0;
    public static double hd = 0.065 , hf = 0.03;

    boolean canon_mode=false;
    public static double canon_velocity=800,ccanon_velocity=800,fcanon_velocity=1200, p = 100, i = 1, d = 10, f = 0.0001, canon_angle = 25;

    PIDFController controller;

    public void init() {
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
        right_canon.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    public void loop(){
        left_canon.setVelocityPIDFCoefficients(p,i,d, f);
        right_canon.setVelocityPIDFCoefficients(p,i,d, f);
        follower.update();
        telemetryM.update();

        double error = targetHeading - follower.getHeading();
        controller.setCoefficients(new PIDFCoefficients(
                hp,
                hi,
                hd,
                hf));
        controller.updateError(error);

        if (headingLock)
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, controller.run(),false);
        else
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x/*+(gamepad1.left_trigger-gamepad1.right_trigger)/2*/,false);

        if (gamepad1.xWasPressed()){headingLock = false;}
        double r2 = gamepad1.right_trigger;
        double l2 = gamepad1.left_trigger;

        if (r2 > 0.05 && l2 <= 0.05) {
            intake.setPower(1);
        }
        else if (l2 > 0.05 && r2 <= 0.05) {
            intake.setPower(-1);
        }
        else {
            intake.setPower(0);      // стоп
        }
        if (gamepad1.right_bumper || gamepad1.x) {
            outtake.setPower(1);}
        else if (gamepad1.left_bumper) {
                outtake.setPower(-1);
            }
        else {
            outtake.setPower(0);
        }


//        if (gamepad1.left_bumper){follower.setPose(new Pose (follower.getPose().getX(),follower.getPose().getY(), Math.toRadians(0)));targetHeading=Math.toRadians(canon_angle);headingLock = true;}

//        if (gamepad1.right_bumper){follower.setPose(new Pose (follower.getPose().getX(),follower.getPose().getY(), Math.toRadians(0)));targetHeading=Math.toRadians(-canon_angle);headingLock = true;}
        if(gamepad1.share){follower.setPose(new Pose (follower.getPose().getX(),follower.getPose().getY(), Math.toRadians(0)));}
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("canon", canon_mode);
        {
            if(canon_mode){left_canon.setVelocity(canon_velocity, AngleUnit.DEGREES);right_canon.setVelocity(canon_velocity, AngleUnit.DEGREES);}
            else{left_canon.setPower(0);right_canon.setPower(0);}
            if(gamepad1.dpadDownWasPressed()){canon_velocity=fcanon_velocity;}
            if(gamepad1.dpadUpWasPressed()){canon_velocity=ccanon_velocity;}
            if (gamepad1.aWasPressed()){canon_mode = !canon_mode;}
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