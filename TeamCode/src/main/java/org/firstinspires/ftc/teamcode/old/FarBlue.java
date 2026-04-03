package org.firstinspires.ftc.teamcode.old;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@Autonomous(name = "FarBlue", group = "Digital")
public class FarBlue extends OpMode {

    /* ================= HARDWARE ================= */
    private DcMotorEx left_canon, right_canon;
    private DcMotorEx intake, outtake;
    private Follower follower;

    /* ================= CONFIG ================= */
    public static double SHOOTER_VELOCITY = 1220;
    public static int TOTAL_SHOTS = 3;

    public static double READY_VELOCITY = 1180;
    public static double DROP_VELOCITY  = 1050;
    public static double angle = 61;

    /* ================= FLAGS ================= */
    private boolean shooterEnabled = true;
    private boolean feederEnabled  = false;
    private boolean intakeEnabled  = true;

    /* ================= SHOOT FSM ================= */
    enum ShootState {
        SPIN_UP,
        READY,
        FIRE,
        RECOVER,
        DONE
    }


    private ShootState shootState = ShootState.SPIN_UP;
    private int shotsDone = 0;

    /* ================= PATH FSM ================= */
    enum PathState {
        START,
        SHOOT_1,
        GO_ART1_START, COLLECT_ART1, RETURN_1,
        SHOOT_2,
        GO_ART2_START, COLLECT_ART2, RETURN_2,
        SHOOT_3,
        GO_ART3_START, COLLECT_ART3, RETURN_3,
        SHOOT_4,
        END,
        GO_AND_COLLECT_ART1,

        GO_AND_COLLECT_ART2,

        GO_AND_COLLECT_ART3,

        OFF
    }
    private PathState pathState = PathState.START;
    private  double breakingPoint = 0.5 ;
    public static double MAX_SHOOT_TIME = 10; // секунд на одну серию
    public static double EJECT_TIME = 0.4  ;
    private final Timer shootTimer = new Timer();
    private final Timer ejectTimer = new Timer();
    private boolean ejectActive = false;


    private final Timer pathTimer = new Timer();

    /* ================= POSES ================= */
    private final Pose startPose =
            new Pose(144-83, 12, Math.toRadians(90));
    private final Pose shootPose =
            new Pose(144-87, 15, Math.toRadians(180-angle));

    private final Pose artifactPoseStart1 =
            new Pose(144-60, 90, Math.toRadians(0));
    private final Pose artifactPoseEnd1 =
            new Pose(144-18, 85.5, Math.toRadians(0));

    private final Pose artifactPoseStart2 =
            new Pose(144-60, 63, Math.toRadians(0));
    private final Pose artifactPoseEnd2 =
            new Pose(144-10, 61.0, Math.toRadians(0));
    private final Pose artifactPoseStart3 =
            new Pose(144-70,45.42596810933941,Math.toRadians(0));
    private final Pose artifactPoseEnd3 =
            new Pose(144-10,42.0,Math.toRadians(0));

    /* ================= PATHS ================= */
    private PathChain startToShoot, end;
    private PathChain toArt1Start, art1Collect, returnToStart1;
    private PathChain toArt2Start, art2Collect, returnToStart2;

    private PathChain toArt3Start, art3Collect, returnToStart3;
    private  PathChain art1Continuous,art2Continuous,art3Continuous;
    /* ================= INIT ================= */
    private boolean delay = true;
    @Override
    public void init() {

        left_canon = hardwareMap.get(DcMotorEx.class, "shooter_l");
        right_canon = hardwareMap.get(DcMotorEx.class, "shooter_r");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        left_canon.setDirection(DcMotor.Direction.FORWARD);
        right_canon.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setDirection(DcMotor.Direction.REVERSE);

        left_canon.setVelocityPIDFCoefficients(10, 0, 0, 13);
        right_canon.setVelocityPIDFCoefficients(10, 0, 0, 13);

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);



        buildPaths();
        pathTimer.resetTimer();
        pathState = PathState.START;

        shooterOff();
        intakeOff();
        shooterReset();
    }

    private void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();

        toArt1Start = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, artifactPoseStart1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), artifactPoseStart1.getHeading())
                .build();

//        art1Collect = follower.pathBuilder()
//                .addPath(new BezierLine(artifactPoseStart1, artifactPoseEnd1))
//                .setLinearHeadingInterpolation(artifactPoseStart1.getHeading(), artifactPoseEnd1.getHeading())
//                .setBrakingStrength(breakingPoint)
//                .build();

        returnToStart1 = follower.pathBuilder()
                .addPath(new BezierLine(artifactPoseEnd1, shootPose))
                .setLinearHeadingInterpolation(artifactPoseEnd1.getHeading(), shootPose.getHeading())
                .build();

        toArt2Start = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, artifactPoseStart2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), artifactPoseStart2.getHeading())
                .build();

//        art2Collect = follower.pathBuilder()
//                .addPath(new BezierLine(artifactPoseStart2, artifactPoseEnd2))
//                .setLinearHeadingInterpolation(artifactPoseStart2.getHeading(), artifactPoseEnd2.getHeading())
//                .setBrakingStrength(breakingPoint)
//                .build();

        returnToStart2 = follower.pathBuilder()
                .addPath(new BezierCurve(artifactPoseEnd2,new Pose(144-34.161731207289293,61.04328018223235,Math.toRadians(0)), shootPose))
                .setLinearHeadingInterpolation(artifactPoseEnd2.getHeading(), shootPose.getHeading())
                .build();
        toArt3Start = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,artifactPoseStart3))
                .setLinearHeadingInterpolation(shootPose.getHeading(),artifactPoseStart3.getHeading())
                .build();
//        art3Collect = follower.pathBuilder()
//                .addPath(new BezierLine(artifactPoseStart3, artifactPoseEnd3))
//                .setLinearHeadingInterpolation(artifactPoseStart3.getHeading(), artifactPoseEnd3.getHeading())
//                .setBrakingStrength(breakingPoint)
//                .build();
        returnToStart3 = follower.pathBuilder()
                .addPath(new BezierCurve(artifactPoseEnd3,new Pose(144-31.161731207289293,65.04328018223235,Math.toRadians(0)), shootPose))
                .setLinearHeadingInterpolation(artifactPoseEnd3.getHeading(), shootPose.getHeading())
                .build();

        art1Continuous = follower.pathBuilder()

                .addPath(new BezierCurve(shootPose, new Pose(144-80,83.1526195899772,Math.toRadians(10)),artifactPoseEnd1))
                .setLinearHeadingInterpolation(shootPose.getHeading(),artifactPoseEnd1.getHeading())
                .setBrakingStrength(0.5)
                .build();

        art2Continuous = follower.pathBuilder()

                .addPath(new BezierCurve(shootPose, new Pose(144-80,58.87927107061503,Math.toRadians(10)),artifactPoseEnd2))
                .setLinearHeadingInterpolation(shootPose.getHeading(),artifactPoseEnd2.getHeading())
                .setBrakingStrength(breakingPoint)
                .build();
        art3Continuous = follower.pathBuilder()

                .addPath(new BezierCurve(shootPose, new Pose(144-80,34.113895216400905,Math.toRadians(10)),artifactPoseEnd3))
                .setLinearHeadingInterpolation(shootPose.getHeading(),artifactPoseEnd3.getHeading())
                .setBrakingStrength(breakingPoint)
                .build();

        end = follower.pathBuilder()

                .addPath(new BezierLine(shootPose,new Pose(144-83, 35, Math.toRadians(90))))
                .setLinearHeadingInterpolation(follower.getHeading(),new Pose(144-10, 64.0, Math.toRadians(90)).getHeading())
                .setBrakingStrength(breakingPoint)
                .build();
    }

    /* ================= LOOP ================= */
    @Override
    public void loop() {
        follower.update();
        updatePathFSM();
        updateShooter();
        telemetry.addData("left_canon_velocity_in_degrees", left_canon.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("right_canon_velocity_in_degrees", right_canon.getVelocity(AngleUnit.DEGREES));
//        updateIntake();
    }

    /* ================= PATH FSM ================= */
    private void updatePathFSM() {

        switch (pathState) {
            case START:
                startPathOnce(startToShoot);
                shooterEnabled = true;

                if (pathTimer.getElapsedTimeSeconds() > 0.5 && !follower.isBusy()) {
                    pathState = PathState.SHOOT_1;
                    pathTimer.resetTimer();
                    delay=true;

                    shooterReset();
                }
                break;


            case SHOOT_1:
                if (updateShootFSM()) {
                    shooterOff();
                    pathState = PathState.END;
                    pathTimer.resetTimer();
                    delay=true;
                }
                break;

//            case GO_ART1_START:
//                intakeOff();
//                startPathOnce(toArt1Start);
//                if (!follower.isBusy()) {
//                    pathState = PathState.COLLECT_ART1;
//                    pathTimer.resetTimer();
//                    delay=true;
//                }
//                break;
//
//            case COLLECT_ART1:
//                intakeOn();
//                startPathOnce(art1Collect);
//                if (!follower.isBusy()) {
////                    intakeOff();
//                    pathState = PathState.RETURN_1;
//                    pathTimer.resetTimer();
//                    delay=true;
//                }
//                break;
            case GO_AND_COLLECT_ART1:
                intakeOn();

                if (delay) {
                    follower.followPath(art1Continuous, true);
                    delay = false;
                }

                if (!follower.isBusy()) {
//                    intakeOff();
                    pathState = PathState.RETURN_1;
                    pathTimer.resetTimer();
                    delay = true;
                }
                break;


            case RETURN_1:

                shooterEnabled = true;
//                intakeOff();
                startPathOnce(returnToStart1);
                if (!follower.isBusy()) {
                    shooterReset();
                    pathState = PathState.SHOOT_2;
                    delay=true;
                }
                break;
            case SHOOT_2:
                if (updateShootFSM()) {
                    shooterOff();
                    pathState = PathState.GO_AND_COLLECT_ART2;
                    pathTimer.resetTimer();
                }
                break;

//            case GO_ART2_START:
//                intakeOn();
//                startPathOnce(toArt2Start);
//                if (!follower.isBusy()) {
//                    pathState = PathState.COLLECT_ART2;
//                    pathTimer.resetTimer();
//                    delay=true;
//                }
//                break;
//
//            case COLLECT_ART2:
//                intakeOn();
//                startPathOnce(art2Collect);
//                if (!follower.isBusy()) {
//                    intakeOff();
//                    pathState = PathState.RETURN_2;
//                    pathTimer.resetTimer();
//                    delay=true;
//                }
//                break;
            case GO_AND_COLLECT_ART2:
                intakeOn();

                if (delay) {
                    follower.followPath(art2Continuous, true);
                    delay = false;
                }

                if (!follower.isBusy()) {
//                    intakeOff();
                    pathState = PathState.RETURN_2;
                    pathTimer.resetTimer();
                    delay = true;
                }
                break;


            case RETURN_2:

                shooterEnabled = true;
//                intakeOff();
                startPathOnce(returnToStart2);
                if (!follower.isBusy()) {
                    shooterReset();
                    pathState = PathState.SHOOT_3;
                    delay=true;
                }
                break;
            case SHOOT_3:
                if (updateShootFSM()) {
                    shooterOff();
                    pathState = PathState.END;
                    pathTimer.resetTimer();
                    delay=true;
                }
                break;
            case GO_ART3_START:
                intakeOn();
                startPathOnce(toArt3Start);
                if (!follower.isBusy()) {
                    pathState = PathState.END;
                    pathTimer.resetTimer();
                    delay=true;
                }
//                break;
//            case COLLECT_ART3:
//                intakeOn();
//                startPathOnce(art3Collect);
//                if (!follower.isBusy()) {
////                    intakeOff();
//                    pathState = PathState.RETURN_3;
//                    pathTimer.resetTimer();
//                    delay=true;
//                }
//                break;
            case GO_AND_COLLECT_ART3:
                intakeOn();

                if (delay) {
                    follower.followPath(art3Continuous, true);
                    delay = false;
                }

                if (!follower.isBusy()) {
//                    intakeOff();
                    pathState = PathState.RETURN_3;
                    pathTimer.resetTimer();
                    delay = true;
                }
                break;


            case RETURN_3:

                shooterEnabled = true;

//                intakeOff();
                startPathOnce(returnToStart3);
                if (!follower.isBusy()) {
                    shooterReset();
                    pathState = PathState.SHOOT_4;
                    delay=true;
                }
                break;
            case SHOOT_4:
                if (updateShootFSM()) {
                    shooterOff();
                    pathState = PathState.END;
                    pathTimer.resetTimer();
                    delay=true;
                }
                break;



            case END:
                shooterOff();
                intakeOff();
                follower.followPath(end, true);
                pathState = PathState.OFF;
                break;
        }
    }

    private void startPathOnce(PathChain path) {
        if(delay){follower.followPath(path, true);delay=false;}
    }
    /* ================= SHOOT FSM (VELOCITY BASED) ================= */
    private boolean updateShootFSM() {

        shooterEnabled = true;

        double v1 = left_canon.getVelocity(AngleUnit.DEGREES);
        double v2 = right_canon.getVelocity(AngleUnit.DEGREES);
        double minV = Math.min(v1, v2);

        switch (shootState) {

            case SPIN_UP:
                feederEnabled = false;
                if (minV > READY_VELOCITY) {
                    shootState = ShootState.READY;
                }
                break;

            case READY:
                feederEnabled = true;
                shootState = ShootState.FIRE;
                break;

            case FIRE:
                feederEnabled = true;
                if (minV < DROP_VELOCITY) {
                    feederEnabled = false;
                    shotsDone++;
                    shootState = ShootState.RECOVER;
                }
                break;

            case RECOVER:
                feederEnabled = false;
                if (minV > READY_VELOCITY) {
                    shootState = ShootState.READY;
                }
                break;

            case DONE:
                shooterEnabled = false;
                feederEnabled = false;
                return true;
        }


        if (shotsDone >= TOTAL_SHOTS) {
            shootState = ShootState.DONE;
        }



        if (shootTimer.getElapsedTimeSeconds() > MAX_SHOOT_TIME) {
            ejectActive = true;
            ejectTimer.resetTimer();
            shootState = ShootState.DONE;
            return true;
        }





        return false;
    }
    private void shooterReset() {
        shootState = ShootState.SPIN_UP;
        shotsDone = 0;
        shootTimer.resetTimer();
        ejectActive = false;
    }



    /* ================= HARDWARE ================= */
    private void updateShooter() {

        // === ШУТЕР ===
        if (shooterEnabled) {
            left_canon.setVelocity(SHOOTER_VELOCITY, AngleUnit.DEGREES);
            right_canon.setVelocity(SHOOTER_VELOCITY, AngleUnit.DEGREES);
        } else {
            left_canon.setVelocity(0);
            right_canon.setVelocity(0);
        }

        outtake.setPower(feederEnabled ? 1 : 0);


        if (ejectActive) {
            intake.setPower(-1);

            if (ejectTimer.getElapsedTimeSeconds() > EJECT_TIME) {
                ejectActive = false;
                intake.setPower(1);
            }
        }
        else if (intakeEnabled || feederEnabled) {
            intake.setPower(1);
        }
        else {
            intake.setPower(1);
        }
    }

    /* ================= HELPERS ================= */
    private void intakeOn()  { intakeEnabled = true; }
    private void intakeOff() { intakeEnabled = false; }

    private void shooterOff() {
        shooterEnabled = false;
        feederEnabled = false;
    }
}