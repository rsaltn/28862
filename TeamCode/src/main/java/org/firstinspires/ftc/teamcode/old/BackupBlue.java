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
@Autonomous(name = "AutoPast", group = "Digital")
public class BackupBlue extends OpMode {

    /* ================= HARDWARE ================= */
    private DcMotorEx shooter1, shooter2;
    private DcMotorEx intakeMotor, take;
    private Follower follower;

    /* ================= CONFIG ================= */
    public static double SHOOTER_VELOCITY = 400;
    public static int TOTAL_SHOTS = 3;

    public static double READY_VELOCITY = 365;
    public static double DROP_VELOCITY  = 340;

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
    }
    private PathState pathState = PathState.START;
    private  double breakingPoint = 0.125;
    public static double MAX_SHOOT_TIME = 5.5; // секунд на одну серию
    public static double EJECT_TIME = 0.4  ;
    private final Timer shootTimer = new Timer();
    private final Timer ejectTimer = new Timer();
    private boolean ejectActive = false;


    private final Timer pathTimer = new Timer();

    /* ================= POSES ================= */
//    private final Pose startPose =
//            new Pose(34.278, 135.144, Math.toRadians(180));
//    private final Pose shootPose =
//            new Pose(31.325740318906607,112.34624145785878,Math.toRadians(137));
//
//    private final Pose artifactPoseStart1 =
//            new Pose(45, 88.5, Math.toRadians(180));
//    private final Pose artifactPoseEnd1 =
//            new Pose(14.5, 84, Math.toRadians(180));
//
//    private final Pose artifactPoseStart2 =
//            new Pose(45, 66, Math.toRadians(180));
//    private final Pose artifactPoseEnd2 =
//            new Pose(8, 61.5, Math.toRadians(180));
//    private final Pose artifactPoseStart3 =
//            new Pose(45,42.42596810933941,Math.toRadians(180));
//    private final Pose artifactPoseEnd3 =
//            new Pose(8,40.42596810933941,Math.toRadians(180));
    private final Pose startPoseR =
            new Pose(144-109.7, 135.144, Math.toRadians(180));
    private final Pose shootPoseR =
            new Pose(144-104.83371298405467,112.47835990888382,Math.toRadians(138));

    private final Pose artifactPoseStart1R =
            new Pose(144-85, 88.5, Math.toRadians(180));
    private final Pose artifactPoseEnd1R =
            new Pose(144-132.5, 84, Math.toRadians(180));

    private final Pose artifactPoseStart2R =
            new Pose(144-85, 66, Math.toRadians(180));
    private final Pose artifactPoseEnd2R =
            new Pose(144-139, 61.5, Math.toRadians(180));
    private final Pose artifactPoseStart3R =
            new Pose(144-85,42.42596810933941,Math.toRadians(180));
    private final Pose artifactPoseEnd3R =
            new Pose(144-139,40.42596810933941,Math.toRadians(180));

    /* ================= PATHS ================= */
    private PathChain startToShoot;
    private PathChain toArt1Start, art1Collect, returnToStart1;
    private PathChain toArt2Start, art2Collect, returnToStart2;

    private PathChain toArt3Start, art3Collect, returnToStart3;
    private  PathChain art1Continuous,art2Continuous,art3Continuous;
    /* ================= INIT ================= */
    private boolean delay = true;
    @Override
    public void init() {

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        take = hardwareMap.get(DcMotorEx.class, "take");

        shooter1.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        take.setDirection(DcMotor.Direction.FORWARD);

        shooter1.setVelocityPIDFCoefficients(10, 0, 0, 12.5);
        shooter2.setVelocityPIDFCoefficients(10, 0, 0, 12.5);

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPoseR);



        buildPaths();
        pathTimer.resetTimer();
        pathState = PathState.START;

        shooterOff();
        intakeOff();
        shooterReset();
    }


    private void buildPaths() {
//        if (isBlue == true){
//            startToShoot = follower.pathBuilder()
//                    .addPath(new BezierLine(startPose,shootPose))
//                    .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
//                    .build();
////
////        toArt1Start = follower.pathBuilder()
////                .addPath(new BezierLine(shootPose, artifactPoseStart1))
////                .setLinearHeadingInterpolation(shootPose.getHeading(), artifactPoseStart1.getHeading())
////                .build();
////
////        art1Collect = follower.pathBuilder()
////                .addPath(new BezierLine(artifactPoseStart1, artifactPoseEnd1))
////                .setLinearHeadingInterpolation(artifactPoseStart1.getHeading(), artifactPoseEnd1.getHeading())
////                .setBrakingStrength(breakingPoint)
////                .build();
//
//            returnToStart1 = follower.pathBuilder()
//                    .addPath(new BezierLine(artifactPoseEnd1, shootPose))
//                    .setLinearHeadingInterpolation(artifactPoseEnd1.getHeading(), shootPose.getHeading())
//                    .build();
//
////        toArt2Start = follower.pathBuilder()
////                .addPath(new BezierLine(shootPose, artifactPoseStart2))
////                .setLinearHeadingInterpolation(shootPose.getHeading(), artifactPoseStart2.getHeading())
////                .build();
////
////        art2Collect = follower.pathBuilder()
////                .addPath(new BezierLine(artifactPoseStart2, artifactPoseEnd2))
////                .setLinearHeadingInterpolation(artifactPoseStart2.getHeading(), artifactPoseEnd2.getHeading())
////                .setBrakingStrength(breakingPoint)
////                .build();
//
//            returnToStart2 = follower.pathBuilder()
//                    .addPath(new BezierCurve(artifactPoseEnd2,new Pose(31.161731207289293,59.04328018223235,Math.toRadians(180)), shootPose))
//                    .setLinearHeadingInterpolation(artifactPoseEnd2.getHeading(), shootPose.getHeading())
//                    .build();
////        toArt3Start = follower.pathBuilder()
////                .addPath(new BezierLine(shootPose,artifactPoseStart3))
////                .setLinearHeadingInterpolation(shootPose.getHeading(),artifactPoseStart3.getHeading())
////                .build();
////        art3Collect = follower.pathBuilder()
////                .addPath(new BezierLine(artifactPoseStart3, artifactPoseEnd3))
////                .setLinearHeadingInterpolation(artifactPoseStart3.getHeading(), artifactPoseEnd3.getHeading())
////                .setBrakingStrength(breakingPoint)
////                .build();
//            returnToStart3 = follower.pathBuilder()
//                    .addPath(new BezierCurve(artifactPoseEnd3,new Pose(31.161731207289293,59.04328018223235,Math.toRadians(180)), shootPose))
//                    .setLinearHeadingInterpolation(artifactPoseEnd3.getHeading(), shootPose.getHeading())
//                    .build();
//
//            art1Continuous = follower.pathBuilder()
//
//                    .addPath(new BezierLine(shootPose, artifactPoseStart1))
//                    .setLinearHeadingInterpolation(
//                            shootPose.getHeading(),
//                            artifactPoseStart1.getHeading()
//                    )
//
//
//                    .addPath(new BezierLine(artifactPoseStart1, artifactPoseEnd1))
//                    .setLinearHeadingInterpolation(
//                            artifactPoseStart1.getHeading(),
//                            artifactPoseEnd1.getHeading()
//                    )
//                    .setBrakingStrength(breakingPoint)
//
//                    .build();
//            art2Continuous = follower.pathBuilder()
//
//                    .addPath(new BezierLine(shootPose, artifactPoseStart2))
//                    .setLinearHeadingInterpolation(
//                            shootPose.getHeading(),
//                            artifactPoseStart2.getHeading()
//                    )
//
//
//                    .addPath(new BezierLine(artifactPoseStart2, artifactPoseEnd2))
//                    .setLinearHeadingInterpolation(
//                            artifactPoseStart2.getHeading(),
//                            artifactPoseEnd2.getHeading()
//                    )
//                    .setBrakingStrength(breakingPoint)
//
//                    .build();
//            art3Continuous = follower.pathBuilder()
//
//                    .addPath(new BezierLine(shootPose, artifactPoseStart3))
//                    .setLinearHeadingInterpolation(
//                            shootPose.getHeading(),
//                            artifactPoseStart3.getHeading()
//                    )
//
//
//                    .addPath(new BezierLine(artifactPoseStart3, artifactPoseEnd3))
//                    .setLinearHeadingInterpolation(
//                            artifactPoseStart3.getHeading(),
//                            artifactPoseEnd3.getHeading()
//                    )
//                    .setBrakingStrength(breakingPoint)
//
//                    .build();}
//        else{
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPoseR,shootPoseR))
                .setLinearHeadingInterpolation(startPoseR.getHeading(),shootPoseR.getHeading())
                .build();
//
//        toArt1Start = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose, artifactPoseStart1))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), artifactPoseStart1.getHeading())
//                .build();
//
//        art1Collect = follower.pathBuilder()
//                .addPath(new BezierLine(artifactPoseStart1, artifactPoseEnd1))
//                .setLinearHeadingInterpolation(artifactPoseStart1.getHeading(), artifactPoseEnd1.getHeading())
//                .setBrakingStrength(breakingPoint)
//                .build();

        returnToStart1 = follower.pathBuilder()
                .addPath(new BezierLine(artifactPoseEnd1R, shootPoseR))
                .setLinearHeadingInterpolation(artifactPoseEnd1R.getHeading(), shootPoseR.getHeading())
                .build();

//        toArt2Start = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose, artifactPoseStart2))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), artifactPoseStart2.getHeading())
//                .build();
//
//        art2Collect = follower.pathBuilder()
//                .addPath(new BezierLine(artifactPoseStart2, artifactPoseEnd2))
//                .setLinearHeadingInterpolation(artifactPoseStart2.getHeading(), artifactPoseEnd2.getHeading())
//                .setBrakingStrength(breakingPoint)
//                .build();

        returnToStart2 = follower.pathBuilder()
                .addPath(new BezierCurve(artifactPoseEnd2R,new Pose(144-112.8,59.04328018223235,Math.toRadians(180)), shootPoseR))
                .setLinearHeadingInterpolation(artifactPoseEnd2R.getHeading(), shootPoseR.getHeading())
                .build();
//        toArt3Start = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose,artifactPoseStart3))
//                .setLinearHeadingInterpolation(shootPose.getHeading(),artifactPoseStart3.getHeading())
//                .build();
//        art3Collect = follower.pathBuilder()
//                .addPath(new BezierLine(artifactPoseStart3, artifactPoseEnd3))
//                .setLinearHeadingInterpolation(artifactPoseStart3.getHeading(), artifactPoseEnd3.getHeading())
//                .setBrakingStrength(breakingPoint)
//                .build();
        returnToStart3 = follower.pathBuilder()
                .addPath(new BezierCurve(artifactPoseEnd3R,new Pose(144-112.8,59.04328018223235,Math.toRadians(180)), shootPoseR))
                .setLinearHeadingInterpolation(artifactPoseEnd3R.getHeading(), shootPoseR.getHeading())
                .build();

        art1Continuous = follower.pathBuilder()

                .addPath(new BezierLine(shootPoseR, artifactPoseStart1R))
                .setLinearHeadingInterpolation(
                        shootPoseR.getHeading(),
                        artifactPoseStart1R.getHeading()
                )


                .addPath(new BezierLine(artifactPoseStart1R, artifactPoseEnd1R))
                .setLinearHeadingInterpolation(
                        artifactPoseStart1R.getHeading(),
                        artifactPoseEnd1R.getHeading()
                )
                .setBrakingStrength(breakingPoint)

                .build();
        art2Continuous = follower.pathBuilder()

                .addPath(new BezierLine(shootPoseR, artifactPoseStart2R))
                .setLinearHeadingInterpolation(
                        shootPoseR.getHeading(),
                        artifactPoseStart2R.getHeading()
                )


                .addPath(new BezierLine(artifactPoseStart2R, artifactPoseEnd2R))
                .setLinearHeadingInterpolation(
                        artifactPoseStart2R.getHeading(),
                        artifactPoseEnd2R.getHeading()
                )
                .setBrakingStrength(breakingPoint)

                .build();
        art3Continuous = follower.pathBuilder()

                .addPath(new BezierLine(shootPoseR, artifactPoseStart3R))
                .setLinearHeadingInterpolation(
                        shootPoseR.getHeading(),
                        artifactPoseStart3R.getHeading()
                )


                .addPath(new BezierLine(artifactPoseStart3R, artifactPoseEnd3R))
                .setLinearHeadingInterpolation(
                        artifactPoseStart3R.getHeading(),
                        artifactPoseEnd3R.getHeading()
                )
                .setBrakingStrength(breakingPoint)

                .build();}




    /* ================= LOOP ================= */
    @Override
    public void loop() {
        follower.update();
        updatePathFSM();
        updateShooter();
        telemetry.addData("left_canon_velocity_in_degrees", shooter1.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("right_canon_velocity_in_degrees", shooter2.getVelocity(AngleUnit.DEGREES));
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
                    pathState = PathState.GO_AND_COLLECT_ART1;
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
                    pathState = PathState.GO_AND_COLLECT_ART3;
                    pathTimer.resetTimer();
                    delay=true;
                }
                break;
//            case GO_ART3_START:
//                intakeOn();
//                startPathOnce(toArt3Start);
//                if (!follower.isBusy()) {
//                    pathState = PathState.;
//                    pathTimer.resetTimer();
//                    delay=true;
//                }
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
                break;
        }
    }

    private void startPathOnce(PathChain path) {
        if(delay){follower.followPath(path, true);delay=false;}
    }

    /* ================= SHOOT FSM (VELOCITY BASED) ================= */
    private boolean updateShootFSM() {

        shooterEnabled = true;

        double v1 = shooter1.getVelocity(AngleUnit.DEGREES);
        double v2 = shooter2.getVelocity(AngleUnit.DEGREES);
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
            shooter1.setVelocity(SHOOTER_VELOCITY, AngleUnit.DEGREES);
            shooter2.setVelocity(SHOOTER_VELOCITY, AngleUnit.DEGREES);
        } else {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
        }

        take.setPower(feederEnabled ? 1 : 0);


        if (ejectActive) {
            intakeMotor.setPower(-1);

            if (ejectTimer.getElapsedTimeSeconds() > EJECT_TIME) {
                ejectActive = false;
                intakeMotor.setPower(1);
            }
        }
        else if (intakeEnabled || feederEnabled) {
            intakeMotor.setPower(1);
        }
        else {
            intakeMotor.setPower(1);
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
