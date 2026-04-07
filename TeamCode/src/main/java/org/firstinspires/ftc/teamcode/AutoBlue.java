package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Auto Blue ", group = "Digital")
public class AutoBlue extends OpMode {
    // ====== Axon PID ======

    /* ================= HARDWARE ================= */
    // простая состояние для стрельбы
    // авто-наведение параметры
    // servo baseline / state
    // pattern of colors to shoot in this autonomous
//    private double breakingPoint = 0.05;
//    private  double br = 1 ;
//    private double maxVelocityPower = 0.6;
    private PathState pathState = PathState.START;
    Timer shootTime = new Timer();
    double veloc = 0;
    public static double angle = -0.68;
    public static double patternID = 1;
    public  static double timeInCheckballs = 0.32;
    public static double[] d = {3730,57};
    public static double SHOOT_DELAY = 0.5; // секунды между transfer'ами
    public static double dxstart = 0;
    public static double dystart = 0;
    public static double dx1 = 0;
    public static double dy1 = 0;
    public static double dx2 = 0;
    public static double dy2 = 0.4;
    public static double dx3 = 0;
    public static double dy3 = -0.3;
    public static double dx4 = 0;
    public static double dy4 = 0;
    Shooter shooter = new Shooter();
    Sorter sorter = new Sorter();
    Servo led;
    CRServo servo;

    Camera cam = new Camera();
    private final Timer shootTimer = new Timer();
    public static double timeForShoot = 1;
    public static double tagHoldSeconds = 0.15;

    private double lastSeenTime = -999;

    private int shootIndex = 0;
    private boolean shootingPhaseActive = false;

    public static double AIM_TX_THRESHOLD = 1.5; // порог «в центре»
    public static double SERVO_GAIN_DIV = 55.0;  // делитель для пропорциональной команды
    public static double MAX_SERVO_DELTA = 0.08; // максимум изменения команды за цикл
    public static double RETURN_SMOOTH = 0.05;   // скорость возвращения к baseline, когда нет метки

    private double servoBaselineCmd = 0.0; // базовая команда — НЕ перезаписываем при переходах
    private double servoLastCmd = 0.0;     // последняя выставленная команда

    public double[] data;
    DcMotorEx intakeF, intakeB, rf, rr, lf, lr;
    private Follower follower;

    /* ============ shooting FSM ============== */
    enum ShootState {
        IDLE,
        START_TRANSFER,
        WAIT_TRANSFER,
        DONE

    }
    private ShootState shootState = ShootState.IDLE;


    public static Sorter.detectedColor[] colors = {Sorter.detectedColor.GREEN, Sorter.detectedColor.PURPLE,  Sorter.detectedColor.PURPLE};

    public static Sorter.detectedColor[] colors1 = {Sorter.detectedColor.GREEN, Sorter.detectedColor.PURPLE,  Sorter.detectedColor.PURPLE};

    public static Sorter.detectedColor[] colors2 = {Sorter.detectedColor.PURPLE, Sorter.detectedColor.GREEN, Sorter.detectedColor.PURPLE};
    public static Sorter.detectedColor[] colors3 = {Sorter.detectedColor.PURPLE, Sorter.detectedColor.PURPLE, Sorter.detectedColor.GREEN};



    /* ================= PATH FSM ================= */
    /* ================= PATH FSM ================= */
    enum PathState {
        START,
        SHOOT_1,
        RETURN_1,
        SHOOT_2,
        RETURN_2,
        RETURN_3,
        RETURN_4,
        RETURN_5,
        SHOOT_3,
        SHOOT_4,
        SHOOT_5,
        SHOOT_6,
        END,
        // split collect states (подход -> collect)
        GO_TO_COLLECT_ART1,
        COLLECT_ART1,
        GO_TO_COLLECT_ART2,
        COLLECT_ART2,
        GO_TO_COLLECT_ART3,
        COLLECT_ART3,
        GO_TO_COLLECT_ART4,
        COLLECT_ART4,
        GO_TO_COLLECT_ART5,
        COLLECT_ART5,
        // old consolidated names kept out of caution: (not used any more)
        GO_AND_COLLECT_ART1,
        GO_AND_COLLECT_ART2,
        GO_AND_COLLECT_ART3,
        GO_AND_COLLECT_ART4,
        GO_AND_COLLECT_ART5,
        GATE1

    }

    private final Timer pathTimer = new Timer();
    private final Timer opTimer = new Timer();

    /* ================= POSES ================= */
    private final Pose startPose = new Pose(28 + dxstart, 131+ dystart, Math.toRadians(143));
    private final Pose shootPose = new Pose(60+ dx1, 84+ dy1, Math.toRadians(180));
    private final Pose artifactPoseEnd1 = new Pose(18.5 + dx1, 84 + dy1, Math.toRadians(180));

    private final Pose artifactPoseStart2 = new Pose(55+ dx2, 58 + dy2, Math.toRadians(180));

    private final Pose artifactPoseEnd2 = new Pose(10+ dx2, 58 + dy2, Math.toRadians(180));
    private final Pose startGate = new Pose(36,72, Math.toRadians(90));
    private final Pose endGate = new Pose(14,72, Math.toRadians(90));
    private final Pose artifactPoseStart3 = new Pose(55 + dx3, 36 + dy3, Math.toRadians(180));
    private final Pose artifactPoseEnd3 = new Pose(10 + dx3, 36 + dy3, Math.toRadians(180));

    private final Pose artifactPoseStart4 = new Pose(40 + dx4, 22 +dy4, Math.toRadians(180));
    private final Pose artifactPoseEnd4 = new Pose(10 + dx4, 22 + dy4, Math.toRadians(180));
    private final Pose artifactPoseStart5 = new Pose(40, 13 , Math.toRadians(180));
    private final Pose artifactPoseEnd5 = new Pose(10, 13,Math.toRadians(180));

    private final Pose End = new Pose(30, 58, Math.toRadians(90));



    /* ================= PATHS ================= */
    // old:
// private PathChain startToShoot, returnToStart1, returnToStart2, returnToStart3,returnToStart4,returnToStart5, art1Continuous, art2Continuous, art3Continuous, art4Continuous, art5Continuous,gate1;

    // new:
    private PathChain startToShoot,
            returnToStart1, returnToStart2, returnToStart3, returnToStart4, returnToStart5,
    // split collect paths: approach -> collect/return
    art1ToCollect, art1FromCollect,
            art2ToCollect, art2FromCollect,
            art3ToCollect, art3FromCollect,
            art4ToCollect, art4FromCollect,
            art5ToCollect, art5FromCollect,
            gate1, end;

    /* ================= INIT ================= */
    private boolean delay = true; // used for starting paths once
    double balls = 0;

    /* ================= Servo aiming helpers ================= */
    private final Timer serv = new Timer();
    public static double delayT = 0.15; // small cooldown for servo adjustments (seconds)
    private boolean servoDelay = false;   // per-servo small cooldown for adjustments

    @Override
    public void init() {


        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

//        Constants.driveConstants.maxPower(maxVelocityPower);

        // hardware
        led = hardwareMap.get(Servo.class, "led");
        // CRServo for aiming
        servo = hardwareMap.get(CRServo.class, "angle");

        cam.init(hardwareMap);

        shooter.init(hardwareMap, true);
        sorter.init(hardwareMap);

        // intake motors
        intakeF = hardwareMap.get(DcMotorEx.class, "intakeF");
        intakeB = hardwareMap.get(DcMotorEx.class, "intakeB");
        intakeF.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeB.setDirection(DcMotorSimple.Direction.FORWARD);

        // drive motors (needed by intakeLogic checks)
        try {
            rf = hardwareMap.get(DcMotorEx.class, "rr");
            rr = hardwareMap.get(DcMotorEx.class, "rf");
            lf = hardwareMap.get(DcMotorEx.class, "lr");
            lr = hardwareMap.get(DcMotorEx.class, "lf");
        } catch (Exception e) {
            rf = rr = lf = lr = null;
        }

        // Запоминаем исходную команду серво (чтобы не «переназначать ноль» позже).
        // Если аппаратно серво только после start(), это всё равно безопасно: baseline остаётся 0.
//        servoBaselineCmd = servo.getPower();
//        servoLastCmd = servoBaselineCmd;


        buildPaths();
        pathTimer.resetTimer();
        opTimer.resetTimer();
        pathState = PathState.START;
        shootState = ShootState.IDLE;
        shootIndex = 0;
        if(patternID == 1){colors = colors1;}
        if(patternID == 2){colors = colors2;}
        if(patternID == 3){colors = colors3;}
    }

    @Override
    public void start() {
        data = shooter.createCanonData(d[0],d[1]);
        shooter.shootON(data);
        serv.resetTimer();
        servoDelay = false;
        servo.setPower(angle);
        opTimer.resetTimer();
        if(patternID == 1){colors = colors1;}
        if(patternID == 2){colors = colors2;}
        if(patternID == 3){colors = colors3;}

        // Оставляем текущее servoLastCmd / servoBaselineCmd.
    }

    private void buildPaths() {
        // start <-> shoot
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .setConstraintsForAll(new PathConstraints(0.99,100,0.2,0.1))
                .build();

        // returnToStart* unchanged
        returnToStart1 = follower.pathBuilder()
                .addPath(new BezierLine(artifactPoseEnd1, shootPose))
                .setLinearHeadingInterpolation(artifactPoseEnd1.getHeading(), shootPose.getHeading())
                .setConstraintsForAll(new PathConstraints(0.98,100,0.2,0.1))
                .build();

        returnToStart2 = follower.pathBuilder()
                .addPath(new BezierCurve(artifactPoseEnd2,new Pose(60,60), shootPose))
                .setLinearHeadingInterpolation(artifactPoseEnd2.getHeading(), shootPose.getHeading())
                .setConstraintsForAll(new PathConstraints(0.98,100,0.2,0.1))
                .build();

        returnToStart3 = follower.pathBuilder()
                .addPath(new BezierCurve(artifactPoseEnd3, new Pose(31.161731207289293, 59.04328018223235, Math.toRadians(180)), shootPose))
                .setLinearHeadingInterpolation(artifactPoseEnd3.getHeading(), shootPose.getHeading())
                .setConstraintsForAll(new PathConstraints(0.98,100,0.2,0.1))
                .build();
        returnToStart4 = follower.pathBuilder()
                .addPath(new BezierCurve(artifactPoseEnd4, new Pose(31.161731207289293, 59.04328018223235, Math.toRadians(180)), shootPose))
                .setLinearHeadingInterpolation(artifactPoseEnd4.getHeading(), shootPose.getHeading())
                .setConstraintsForAll(new PathConstraints(0.98,100,0.2,0.1))
                .build();
        returnToStart5 = follower.pathBuilder()
                .addPath(new BezierCurve(artifactPoseEnd5, new Pose(31.161731207289293, 59.04328018223235, Math.toRadians(180)), shootPose))
                .setLinearHeadingInterpolation(artifactPoseEnd5.getHeading(), shootPose.getHeading())
                .setConstraintsForAll(new PathConstraints(0.98,100,0.2,0.1))
                .build();

        // ----------------------------
        // ART 1: split
        art1ToCollect = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, artifactPoseEnd1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), artifactPoseEnd1.getHeading())
//                // slow / precise collect approach:
//                .setBrakingStrength(br)
//                .setBrakingStart(br)
                .setConstraintsForAll(new PathConstraints(0.995, 0.5, 0.5, 0.03, 70, 0.17, 20, 0.7))

                .setVelocityConstraint(0.1)
                .build();


        // ----------------------------
        // ART 2: split (originally had two segments: shoot->start2, start2->end2)
        art2ToCollect = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, artifactPoseStart2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), artifactPoseStart2.getHeading())
//                .setConstraintsForAll(new PathConstraints(0.995, 0.5, 0.5, 0.03, 70, 0.17, 20, 0.4))

                .build();

        art2FromCollect = follower.pathBuilder()
                .addPath(new BezierLine(artifactPoseStart2, artifactPoseEnd2))
                .setLinearHeadingInterpolation(artifactPoseStart2.getHeading(), artifactPoseEnd2.getHeading())
//                .setBrakingStrength(br)
//                .setBrakingStart(br)
                .setConstraintsForAll(new PathConstraints(0.995, 0.5, 0.5, 0.03, 70, 0.17, 20, 0.7))


                .setVelocityConstraint(0.1)
                .build();


        // ----------------------------
        // gate path (unchanged)
        gate1 = follower.pathBuilder()
                .addPath(new BezierCurve(artifactPoseEnd2, new Pose(30.48519362186788, 65.32346241457861, Math.toRadians(180)), startGate, endGate))
                .setLinearHeadingInterpolation(artifactPoseEnd2.getHeading(), endGate.getHeading())
//                .setBrakingStrength(breakingPoint)
                .addPath(new BezierLine(endGate, shootPose))
                .setLinearHeadingInterpolation(endGate.getHeading(), shootPose.getHeading())

                .setVelocityConstraint(0.1)
                .build();

        // ----------------------------
        // ART 3
        art3ToCollect = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, artifactPoseStart3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), artifactPoseStart3.getHeading())

                .build();

        art3FromCollect = follower.pathBuilder()
                .addPath(new BezierLine(artifactPoseStart3, artifactPoseEnd3))
                .setLinearHeadingInterpolation(artifactPoseStart3.getHeading(), artifactPoseEnd3.getHeading())
//                .setBrakingStrength(br)
//                .setBrakingStart
                .setConstraintsForAll(new PathConstraints(0.995, 0.5, 0.5, 0.03, 70, 0.17, 20, 0.7))


                .setVelocityConstraint(0.1)
                .build();


        // ----------------------------
        // ART 4
        art4ToCollect = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, artifactPoseStart4))
                .setLinearHeadingInterpolation(shootPose.getHeading(), artifactPoseStart4.getHeading())

                .build();

        art4FromCollect = follower.pathBuilder()
                .addPath(new BezierLine(artifactPoseStart4, artifactPoseEnd4))
                .setLinearHeadingInterpolation(artifactPoseStart4.getHeading(), artifactPoseEnd4.getHeading())
//                .setBrakingStrength(br)
//                .setBrakingStart(br)
                .setConstraintsForAll(new PathConstraints(0.995, 0.5, 0.5, 0.03, 70, 0.17, 20, 0.7))


                .setVelocityConstraint(0.1)
                .build();

        // ----------------------------
        // ART 5
        art5ToCollect = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, artifactPoseStart5))
                .setLinearHeadingInterpolation(shootPose.getHeading(), artifactPoseStart5.getHeading())

                .build();

        art5FromCollect = follower.pathBuilder()
                .addPath(new BezierLine(artifactPoseStart5, artifactPoseEnd5))
                .setLinearHeadingInterpolation(artifactPoseStart5.getHeading(), artifactPoseEnd5.getHeading())
//                .setBrakingStrength(br)
//                .setBrakingStart(br)
                .setConstraintsForAll(new PathConstraints(0.995, 0.5, 0.5, 0.03, 70, 0.17, 20, 0.7))


                .setVelocityConstraint(0.1)
                .build();



    }


    /* ================= LOOP ================= */
    @Override
    public void loop() {
        if(patternID == 1){colors = colors1;}
        if(patternID == 2){colors = colors2;}
        if(patternID == 3){colors = colors3;}

        double now = getRuntime();

        follower.update();
        updatePathFSM();      // path FSM (calls shooting() inside SHOOT states)
        cam.update();         // update camera after shooter to keep timing consistent

        // tag "hold" logic
        if (cam.hasTag) {
            lastSeenTime = now;
        }
        boolean haveRecentTag = (now - lastSeenTime) <= tagHoldSeconds;

        // Auto-aim: только в стадиях стрельбы и только если камера видит метку



        // sorter FSM must run every loop
        shooter.shootON(data);
        sorter.updateTransfer();
        telemetry.addData("Shooter target RPM", shooter.getTargetRpm());
        telemetry.addData("Shooter meas RPM", shooter.getMeasuredRpm());
        telemetry.addData("PathState: ", pathState);
        telemetry.update();
    }

    private boolean isInShootingStage() {
        return pathState == PathState.SHOOT_1
                || pathState == PathState.SHOOT_2
                || pathState == PathState.SHOOT_3
                || pathState == PathState.SHOOT_4;
    }

    private void updatePathFSM() {

        switch (pathState) {
            case START:
                leave();
                startPathOnce(startToShoot);
                data = shooter.createCanonData(d[0],d[1]);

                if (pathTimer.getElapsedTimeSeconds() > 0.5 && !follower.isBusy()) {
                    pathState = PathState.SHOOT_1;
                    pathTimer.resetTimer();
                    delay = true;

                    // reset shooting FSM for the new shooting phase
                    shootState = ShootState.IDLE;
                    shootIndex = 0;
                    shootTime.resetTimer();
                }
                break;

            case SHOOT_1:
                leave();
                intakeF.setPower(0);
                intakeB.setPower(0);


                // запускаем простую логику стрельбы
                shooting();

                // переход только если стрельба явно завершена (нет активного transfer'а и слоты пусты)
                if ((!shootingPhaseActive && !sorter.transferActive && checkBalls()) || shootTime.getElapsedTimeSeconds() > 6) {
                    shooterOff();
                    pathState = PathState.GO_TO_COLLECT_ART1; // <- now go to first split state
                    pathTimer.resetTimer();
                    delay = true;

                }
                break;

            // ---- GO_TO_COLLECT_ART1 -> COLLECT_ART1 ----
            case GO_TO_COLLECT_ART1:
                leave();
                intakeLogic();

                if (delay) {
//                    Constants.driveConstants.xVelocity(10);
                    follower.followPath(art1ToCollect, true);
                    delay = false;
                }

                if (!follower.isBusy()) {

//                    Constants.driveConstants.xVelocity(90);
                    pathState = PathState.RETURN_1;
                    pathTimer.resetTimer();
                    delay = true;
                }
                break;



            // ---- RETURN_1 (unchanged) ----
            case RETURN_1:
                leave();
                data = shooter.createCanonData(d[0],d[1]);

                startPathOnce(returnToStart1);
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_2;
                    delay = true;

                    // reset shooting FSM for the next shooting phase
                    shootState = ShootState.IDLE;
                    shootIndex = 0;
                    shootTime.resetTimer();
                }
                break;

            case SHOOT_2:
                leave();
                intakeF.setPower(0);
                intakeB.setPower(0);

                shooting();
                if ((!shootingPhaseActive && !sorter.transferActive && checkBalls()) || shootTime.getElapsedTimeSeconds() > 6) {
                    shooterOff();
                    pathState = PathState.GO_TO_COLLECT_ART2; // split here
                    pathTimer.resetTimer();
                }
                break;

            // GO_TO_COLLECT_ART2 -> COLLECT_ART2
            case GO_TO_COLLECT_ART2:
                leave();
                intakeLogic();

                if (delay) {

                    follower.followPath(art2ToCollect, true);
                    delay = false;
                }

                if (!follower.isBusy()) {
                    pathState = PathState.COLLECT_ART2;
                    pathTimer.resetTimer();
                    delay = true;
                }
                break;

            case COLLECT_ART2:
                leave();
                intakeLogic();

                if (delay) {

//                    Constants.driveConstants.xVelocity(10);
                    follower.followPath(art2FromCollect, true);
                    delay = false;
                }

                if (!follower.isBusy()) {

                    pathState = PathState.RETURN_2;
                    pathTimer.resetTimer();
                    delay = true;
                }
                break;

            // GATE1 unchanged
            case GATE1:
                leave();
                if (delay) {
                    follower.followPath(gate1, true);
                    delay = false;
                }

                if (!follower.isBusy()) {
                    pathState = PathState.RETURN_2;
                    pathTimer.resetTimer();
                    delay = true;
                }
                break;

            // RETURN_2 unchanged (keeps existing logic)
            case RETURN_2:
                leave();
                data = shooter.createCanonData(d[0],d[1]);
                startPathOnce(returnToStart2);
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_3;
                    delay = true;

                    // reset shooting FSM
                    shootState = ShootState.IDLE;
                    shootIndex = 0;
                    shootTime.resetTimer();
                }
                break;

            // SHOOT_3 -> GO_TO_COLLECT_ART3 -> COLLECT_ART3
            case SHOOT_3:
                leave();
                intakeF.setPower(0);
                intakeB.setPower(0);
                shooting();
                if ((!shootingPhaseActive && !sorter.transferActive && checkBalls()) || shootTime.getElapsedTimeSeconds() > 6) {
                    shooterOff();
                    pathState = PathState.GO_TO_COLLECT_ART3;
                    pathTimer.resetTimer();
                    delay = true;
                }
                break;

            case GO_TO_COLLECT_ART3:
                leave();
                intakeLogic();

                if (delay) {
                    follower.followPath(art3ToCollect, true);
                    delay = false;
                }

                if (!follower.isBusy()) {
                    pathState = PathState.COLLECT_ART3;
                    pathTimer.resetTimer();
                    delay = true;
                }
                break;

            case COLLECT_ART3:
                leave();
                intakeLogic();

                if (delay) {
                    follower.followPath(art3FromCollect, true);
                    delay = false;
                }

                if (!follower.isBusy()) {
                    pathState = PathState.RETURN_3;
                    pathTimer.resetTimer();
                    delay = true;
                }
                break;

            // RETURN_3 (unchanged)
            case RETURN_3:
                leave();
                data = shooter.createCanonData(d[0],d[1]);
                startPathOnce(returnToStart3);
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_4;
                    delay = true;
                    shootTime.resetTimer();
                }
                break;

            // SHOOT_4 -> GO_TO_COLLECT_ART4 -> COLLECT_ART4
            case SHOOT_4:
                leave();
                intakeF.setPower(0);
                intakeB.setPower(0);
                shooting();
                if ((!shootingPhaseActive && !sorter.transferActive && checkBalls()) || shootTime.getElapsedTimeSeconds() > 6) {
                    shooterOff();
                    pathState = PathState.GO_TO_COLLECT_ART4;
                    pathTimer.resetTimer();
                    delay = true;
                }
                break;

            case GO_TO_COLLECT_ART4:
                leave();
                intakeLogic();

                if (delay) {
                    follower.followPath(art4ToCollect, true);
                    delay = false;
                }

                if (!follower.isBusy()) {
                    pathState = PathState.COLLECT_ART4;
                    pathTimer.resetTimer();
                    delay = true;
                }
                break;

            case COLLECT_ART4:
                leave();
                intakeLogic();

                if (delay) {
                    follower.followPath(art4FromCollect, true);
                    delay = false;
                }

                if (!follower.isBusy()) {
                    pathState = PathState.RETURN_4;
                    pathTimer.resetTimer();
                    delay = true;
                }
                break;

            // RETURN_4 unchanged
            case RETURN_4:
                leave();
                data = shooter.createCanonData(d[0],d[1]);
                startPathOnce(returnToStart4);
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_5;
                    delay = true;

                    // reset shooting FSM
                    shootState = ShootState.IDLE;
                    shootIndex = 0;
                }
                break;

            // SHOOT_5 -> GO_TO_COLLECT_ART5 -> COLLECT_ART5
            case SHOOT_5:
                leave();
                intakeF.setPower(0.4);
                intakeB.setPower(0.4);

                shooting();
                if (!shootingPhaseActive && !sorter.transferActive && checkBalls()) {
                    shooterOff();
                    pathState = PathState.GO_TO_COLLECT_ART5;
                    pathTimer.resetTimer();
                }
                break;

            case GO_TO_COLLECT_ART5:
                leave();
                intakeLogic();

                if (delay) {
                    follower.followPath(art5ToCollect, true);
                    delay = false;
                }

                if (!follower.isBusy()) {
                    pathState = PathState.COLLECT_ART5;
                    pathTimer.resetTimer();
                    delay = true;
                }
                break;

            case COLLECT_ART5:
                leave();
                intakeLogic();

                if (delay) {
                    follower.followPath(art5FromCollect, true);
                    delay = false;
                }

                if (!follower.isBusy()) {
                    pathState = PathState.RETURN_5;
                    pathTimer.resetTimer();
                    delay = true;
                }
                break;

            // RETURN_5 and SHOOT_6 / END remain identical to previous implementation
            case RETURN_5:
                leave();
                data = shooter.createCanonData(d[0],d[1]);
                startPathOnce(returnToStart5);
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_6;
                    delay = true;

                    // reset shooting FSM
                    shootState = ShootState.IDLE;
                    shootIndex = 0;
                }
                break;

            case SHOOT_6:
                leave();
                intakeF.setPower(0.4);
                intakeB.setPower(0.4);

                shooting();
                if (!shootingPhaseActive && !sorter.transferActive && checkBalls()) {
                    shooterOff();
                    pathState = PathState.END;
                    pathTimer.resetTimer();
                }
                break;

            case END:
                shooterOff();
                break;
        }
    }

    private void startPathOnce(PathChain path) {
        if (delay) {
            follower.followPath(path, true);
            delay = false;
        }
    }

    private void shooterOff() {
        data = shooter.createCanonData(0, 0);
    }

    private void intakeLogic() {
        if (lr.getPower() > 0.1 || lf.getPower() > 0.1 || rf.getPower() > 0.1 || rr.getPower() > 0.1) {
            intakeF.setPower(1);
            intakeB.setPower(-0.2);

        } else {
            intakeF.setPower(0.9);
            intakeB.setPower(-0.6);
        }
    }


    private boolean checkBalls() {
        balls = 0;
        waitS(timeInCheckballs);
        if (sorter.slotL.getBallColor() != Sorter.detectedColor.NOTHING) {
            balls++;
        }
        if (sorter.slotRF.getBallColor() != Sorter.detectedColor.NOTHING) {
            balls++;
        }
        if (sorter.slotRR.getBallColor() != Sorter.detectedColor.NOTHING) {
            balls++;
        }

        return balls == 0;
    }


    private void shooting() {
        //intakeLogic();
        sorter.updateTransfer();

        if (!shootingPhaseActive ) {
            shootingPhaseActive = true;
            shootIndex = 0;
            shootTimer.resetTimer();
        }

        // Если слотов пусто и трансфер НЕ активен — считаем фазу завершённой
        if (checkBalls() && !sorter.transferActive) {
            shootingPhaseActive = false;
            shootIndex = 0;
            return;
        }

        if (sorter.transferActive) {
            return;
        }

        if (shootIndex < colors.length) {
            if (shootTimer.getElapsedTimeSeconds() >= SHOOT_DELAY ) {
                sorter.startTransfer(colors[shootIndex]);
                shootIndex++;
                shootTimer.resetTimer();
            }
            return;
        }

        if (!checkBalls()) {
            shootIndex = 0;
            shootTimer.resetTimer();
        } else {
            if (!sorter.transferActive) {
                shootingPhaseActive = false;
                shootIndex = 0;
            }
        }
    }
    public void waitS(double s)
    {
        Timer timer = new Timer();
        timer.resetTimer();
        while(timer.getElapsedTimeSeconds()<s)
        {
            shooter.shootON(data);
            follower.update();
        }
    }
    public void leave()
    {
        if(opTimer.getElapsedTimeSeconds()>28)
        {
            end = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(),End))
                    .setLinearHeadingInterpolation(follower.getHeading(),End.getHeading())
                    .build();
            follower.followPath(end);
            pathState = PathState.END;
        }
    }
}

