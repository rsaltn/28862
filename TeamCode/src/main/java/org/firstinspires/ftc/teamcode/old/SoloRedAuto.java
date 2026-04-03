package org.firstinspires.ftc.teamcode.old;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled

@Autonomous(name = "SoloRedAuto", group = "Digital")
public class SoloRedAuto extends OpMode {

    private Follower follower;

    DcMotorEx left_canon, right_canon,intake;

    private TelemetryManager telemetryM;

    public int shot=0;

    public boolean wait=false;
    public static double canon_velocity=1220, p = 5000, i = 1, d = 10, f = 0.0001, shoot_time = 1.5;
    CRServo outtake;
    private Timer pathTimer, actionTimer, opmodeTimer;


    private int pathState;
    private final Pose firststartpose = new Pose(83, 12, Math.toRadians(90));
    private final Pose startPose = new Pose(87, 15, Math.toRadians(68));
    private final Pose controlPoint1 = new Pose(80, 27, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(100, 27, Math.toRadians(180));


    private final Pose controlPoint2 = new Pose(90,25 , Math.toRadians(0));

    private final Pose shoot1Pose = new Pose(78, 66, Math.toRadians(45));
    private final Pose endPose = new Pose(125, -8, Math.toRadians(270));

    

    private PathChain startingPath, grabPickup1,grabPickup2,shootPath1,shootPath2, endPath;


    public void buildPaths() {

        startingPath = follower.pathBuilder()
                .addPath(new BezierLine(firststartpose, startPose))
                .setLinearHeadingInterpolation(firststartpose.getHeading(), startPose.getHeading())
                .build();


        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,controlPoint1))
                .setLinearHeadingInterpolation(startPose.getHeading(),controlPoint1.getHeading())
                .setBrakingStrength(0.1)
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(controlPoint1,pickup1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(),controlPoint1.getHeading())
                .setBrakingStrength(0.1)
                .build();


        shootPath1 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose, startPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), startPose.getHeading())
                .build();


        endPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(follower.getPose().getX(),follower.getPose().getY(), follower.getHeading()),controlPoint2, endPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), endPose.getHeading())
                .build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                left_canon.setVelocity(canon_velocity, AngleUnit.DEGREES);
                right_canon.setVelocity(canon_velocity, AngleUnit.DEGREES);
                intake.setPower(-0.7);


                if((left_canon.getVelocity(AngleUnit.DEGREES) < canon_velocity + 20 && left_canon.getVelocity(AngleUnit.DEGREES) > canon_velocity-20 )&&(right_canon.getVelocity(AngleUnit.DEGREES) < canon_velocity + 20 && right_canon.getVelocity(AngleUnit.DEGREES) > canon_velocity-20 )&&(pathTimer.getElapsedTimeSeconds()%shoot_time >= shoot_time/4)&&!wait){
                    wait = true;
                    shot+=1;
                    outtake.setPower(-1);}
                else if((pathTimer.getElapsedTimeSeconds()%shoot_time <= shoot_time/4)){
                    wait=false;
                    outtake.setPower(0.1);}

                if(shot >=4 && !wait){
                    shot=0;
                    outtake.setPower(1);
                    follower.followPath(grabPickup1);
                    setPathState(-1);
                    break;
                }

                break;

            case 1:
                intake.setPower(-1);
                if (!follower.isBusy())  {
                    outtake.setPower(1);
                    follower.followPath(grabPickup2);
                    setPathState(2);
                    break;
                }
            case 2:
                intake.setPower(-1);
                if (!follower.isBusy())  {
                    outtake.setPower(1);
                    if(pathTimer.getElapsedTimeSeconds() > 3){
                        follower.followPath(shootPath1);
                        setPathState(3);
                        break;
                    }
                    break;
                }

            case 3:
                intake.setPower(-1);
                if (!follower.isBusy()) {
                    if((left_canon.getVelocity(AngleUnit.DEGREES) < canon_velocity + 20 && left_canon.getVelocity(AngleUnit.DEGREES) > canon_velocity-20 )&&(right_canon.getVelocity(AngleUnit.DEGREES) < canon_velocity + 20 && right_canon.getVelocity(AngleUnit.DEGREES) > canon_velocity-20 )&&(pathTimer.getElapsedTimeSeconds()%shoot_time >= shoot_time/4)&&!wait){
                        wait = true;
                        shot+=1;
                        outtake.setPower(-1);}
                    else if((pathTimer.getElapsedTimeSeconds()%shoot_time <= shoot_time/4)){
                        wait=false;
                        outtake.setPower(0);}

                    if(shot >=4 && !wait){
                        shot=0;
                        outtake.setPower(1);
                        follower.followPath(grabPickup1);
                        setPathState(1);
                        break;
                    }
                }
                break;
        }
    }


    public void setPathState(int pState) {
        intake.setPower(-1);
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();
        intake.setPower(-1);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("left_canon_velocity_in_degrees", left_canon.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("right_canon_velocity_in_degrees", right_canon.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("shot", shot);
        telemetry.update();
        telemetryM.addData("path state", pathState);
        telemetryM.addData("x", follower.getPose().getX());
        telemetryM.addData("y", follower.getPose().getY());
        telemetryM.addData("heading", follower.getPose().getHeading());
        telemetry.addData("left_canon_velocity_in_degrees", left_canon.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("right_canon_velocity_in_degrees", right_canon.getVelocity(AngleUnit.DEGREES));
        telemetryM.addData("shot", shot);
        telemetryM.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        left_canon = hardwareMap.get(DcMotorEx.class, "left_canon");
        right_canon = hardwareMap.get(DcMotorEx.class, "right_canon");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(CRServo.class, "servo_launch");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


        left_canon.setDirection(DcMotorSimple.Direction.REVERSE);


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(firststartpose);

    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        follower.followPath(startingPath);
        opmodeTimer.resetTimer();
        setPathState(0);
        left_canon.setVelocityPIDFCoefficients(p,i,d,f);
        right_canon.setVelocityPIDFCoefficients(p,i,d,f);
    }
    @Override
    public void stop() {
    }
}