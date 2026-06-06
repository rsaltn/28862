package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Auto Blue + cam ", group = "Digital")
public class AutoBlueCam extends OpMode {

    // ========== НАСТРОЙКИ ==========
    public static double[] lockerStates = {0.7, 1.0};      // 0 = открыт, 1 = закрыт
    public static double TIME_TO_SHOOT         = 0.6;
    public static double TIME_FOR_GATE_INTAKE  = 0.6;      // задержка после сбора
    public static double[] shootRPMangle       = {3900, 160};

    public static double INTAKE_POWER_COLLECT  = 1.0;
    public static double INTAKE_POWER_HOLD     = 0.7;
    public static double INTAKE_POWER_SHOOT    = 1.0;

    public static double CAMERA_TX_SIGN  = 1.0;
    public static double DEADBAND_DEG    = 1.2;
    public static double SERVO_GAIN      = 0.0175;
    public static double SERVO_MAX_POWER = 1;

    // BUG FIX: время аварийного выхода 28 → 29 секунд
    public static double EMERGENCY_EXIT_TIME = 29.0;

    // ========== ОБОРУДОВАНИЕ ==========
    private Follower follower;
    private CRServo locker;
    private CRServo angleServo;
    private DcMotorEx intake_l, intake_r;
    private Shooter shooter = new Shooter();
    private Camera  cam     = new Camera();

    // ========== АВТОМАТ СОСТОЯНИЙ ==========
    private enum PathState {
        START, SHOOT_1, GO_TO_COLLECT_1, COLLECT_1, RETURN_1,
        SHOOT_2, TO_GATE, GATE, GO_TO_COLLECT_2, RETURN_2,
        SHOOT_3, COLLECT_2, RETURN_3,
        SHOOT_4, GO_TO_COLLECT_3, COLLECT_3, RETURN_4,
        SHOOT_5, END
    }
    private PathState pathState  = PathState.START;
    private Timer     stateTimer = new Timer();
    // BUG FIX: отдельный таймер для ожидания после гейта,
    //          чтобы не зависеть от stateTimer который используется для стрельбы
    private Timer gateWaitTimer  = new Timer();
    private boolean firstEnter   = true;
    private boolean emergencyExit  = false;
    private boolean endPathBuilt   = false;
    private boolean gatePathDone   = false;   // флаг: путь завершён, идём таймер

    // Пути
    private PathChain startToShoot, toCollect1, collect1, return1,
            toGate, gate, toCollect2, return2,
            collect2, return3, toCollect3, collect3, return4,
            endPath;

    private final Pose startPose = new Pose(29.000, 133.000, Math.toRadians(143));
    private final Pose endPose   = new Pose(30, 58, Math.toRadians(90));

    private double[] shootData;
    private boolean  aimingActive = false;

    // ========== ИНИЦИАЛИЗАЦИЯ ==========
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        locker     = hardwareMap.get(CRServo.class, "locker");
        angleServo = hardwareMap.get(CRServo.class, "angle");
        intake_l   = hardwareMap.get(DcMotorEx.class, "intake_l");
        intake_r   = hardwareMap.get(DcMotorEx.class, "intake_r");

        // На старте локер должен быть закрыт, чтобы мячи не выпадали
        locker.setPower(lockerStates[1]);   // закрыт

        shooter.init(hardwareMap, true);
        cam.init(hardwareMap);

        buildPaths();
        stateTimer.resetTimer();
    }
    @Override
    public void start() {
        // Дополнительная страховка: закрываем локер перед началом движения
        locker.setPower(lockerStates[1]);

        shootData = shooter.createCanonData(shootRPMangle[0], shootRPMangle[1]);
        shooter.shootON(shootData);
        stateTimer.resetTimer();
        firstEnter = true;
        setIntakePower(INTAKE_POWER_HOLD);
        angleServo.setPower(0);
        aimingActive = false;
    }


    // ========== ОСНОВНОЙ ЦИКЛ ==========
    @Override
    public void loop() {
        cam.update();
        follower.update();
        updateFSM();

        if (aimingActive) {
            performAiming();
        }

        shooter.shootON(shootData);

        telemetry.addData("State",              pathState);
        telemetry.addData("Runtime",            getRuntime());
        telemetry.addData("Intake power",       intake_l.getPower());
        telemetry.addData("AngleServo power",   angleServo.getPower());
        telemetry.addData("cam.hasTag",         cam.hasTag);
        telemetry.addData("cam.tx",             cam.tx);
        telemetry.addData("Shooter target RPM", shooter.getTargetRpm());
        telemetry.addData("Shooter meas RPM",   shooter.getMeasuredRpm());
        telemetry.update();
    }

    // ========== АВТОНАВОДКА ==========
    private void performAiming() {
        if (!cam.hasTag) return;
        double headingError = -cam.tx * CAMERA_TX_SIGN;
        if (Math.abs(headingError) <= DEADBAND_DEG) {
            angleServo.setPower(0);
        } else {
            angleServo.setPower(Range.clip(headingError * SERVO_GAIN, -SERVO_MAX_POWER, SERVO_MAX_POWER));
        }
    }

    // ========== ИНТЕЙК ==========
    private void setIntakePower(double power) {
        power = Range.clip(power, -1.0, 1.0);
        intake_l.setPower(power);
        intake_r.setPower(power);
    }
    private void startIntakeFull()  { setIntakePower(INTAKE_POWER_COLLECT); }
    private void startIntakeHold()  { setIntakePower(INTAKE_POWER_HOLD);    }
    private void startIntakeShoot() { setIntakePower(INTAKE_POWER_SHOOT);   }
    private void stopIntake()       { setIntakePower(0);                     }

    // ========== FSM ==========
    private void updateFSM() {
        // BUG FIX: аварийный выход в 29 сек (было 28), константа вынесена в @Configurable
        if (!emergencyExit && getRuntime() > EMERGENCY_EXIT_TIME) {
            emergencyExit = true;
            if (!endPathBuilt) {
                endPath = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), endPose))
                        .setLinearHeadingInterpolation(follower.getHeading(), endPose.getHeading())
                        .build();
                endPathBuilt = true;
                follower.followPath(endPath);   // BUG FIX: followPath внутри блока построения,
            }                                   //          чтобы не вызываться каждый луп
            pathState = PathState.END;
        }

        switch (pathState) {

            case START:
                if (firstEnter) {
                    follower.followPath(startToShoot, true);
                    firstEnter = false;
                    stateTimer.resetTimer();
                    startIntakeHold();
                    aimingActive = false;
                }
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() > 0.5) {
                    pathState  = PathState.SHOOT_1;
                    firstEnter = true;
                }
                break;

            case SHOOT_1:
                if (firstEnter) {
                    startIntakeShoot();
                    locker.setPower(lockerStates[0]);
                    stateTimer.resetTimer();
                    aimingActive = true;
                    firstEnter   = false;
                }
                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT) {
                    stopIntake();
                    locker.setPower(lockerStates[1]);
                    aimingActive = false;
                    angleServo.setPower(0);
                    pathState  = PathState.GO_TO_COLLECT_1;
                    firstEnter = true;
                }
                break;

            case GO_TO_COLLECT_1:
                if (firstEnter) {
                    startIntakeFull();
                    follower.followPath(toCollect1, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState  = PathState.COLLECT_1;
                    firstEnter = true;
                }
                break;

            case COLLECT_1:
                if (firstEnter) {
                    startIntakeFull();
                    follower.followPath(collect1, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState  = PathState.RETURN_1;
                    firstEnter = true;
                }
                break;

            case RETURN_1:
                if (firstEnter) {
                    startIntakeHold();
                    follower.followPath(return1, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState  = PathState.SHOOT_2;
                    firstEnter = true;
                }
                break;

            case SHOOT_2:
                if (firstEnter) {
                    startIntakeShoot();
                    locker.setPower(lockerStates[0]);
                    stateTimer.resetTimer();
                    aimingActive = true;
                    firstEnter   = false;
                }
                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT) {
                    stopIntake();
                    locker.setPower(lockerStates[1]);
                    aimingActive = false;
                    angleServo.setPower(0);
                    pathState  = PathState.TO_GATE;
                    firstEnter = true;
                }
                break;

            case TO_GATE:
                if (firstEnter) {
                    startIntakeHold();
                    follower.followPath(toGate, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState  = PathState.GATE;
                    firstEnter = true;
                }
                break;

            case GATE:
                if (firstEnter) {
                    follower.followPath(gate, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState  = PathState.GO_TO_COLLECT_2;
                    firstEnter = true;
                }
                break;

            case GO_TO_COLLECT_2:
                if (firstEnter) {
                    startIntakeFull();
                    follower.followPath(toCollect2, true);
                    gatePathDone = false;
                    firstEnter   = false;
                }
                // BUG FIX: добавлено двухэтапное ожидание:
                //   1) ждём конца пути
                //   2) ждём ещё TIME_FOR_GATE_INTAKE секунд (чтобы мячи успели попасть в интейк)
                if (!follower.isBusy() && !gatePathDone) {
                    gateWaitTimer.resetTimer();
                    gatePathDone = true;
                }
                if (gatePathDone && gateWaitTimer.getElapsedTimeSeconds() > TIME_FOR_GATE_INTAKE) {
                    pathState  = PathState.RETURN_2;
                    firstEnter = true;
                }
                break;

            case RETURN_2:
                if (firstEnter) {
                    startIntakeHold();
                    follower.followPath(return2, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState  = PathState.SHOOT_3;
                    firstEnter = true;
                }
                break;

            case SHOOT_3:
                if (firstEnter) {
                    startIntakeShoot();
                    locker.setPower(lockerStates[0]);
                    stateTimer.resetTimer();
                    aimingActive = true;
                    firstEnter   = false;
                }
                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT) {
                    stopIntake();
                    locker.setPower(lockerStates[1]);
                    aimingActive = false;
                    angleServo.setPower(0);
                    pathState  = PathState.COLLECT_2;
                    firstEnter = true;
                }
                break;

            case COLLECT_2:
                if (firstEnter) {
                    startIntakeFull();
                    follower.followPath(collect2, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState  = PathState.RETURN_3;
                    firstEnter = true;
                }
                break;

            case RETURN_3:
                if (firstEnter) {
                    startIntakeHold();
                    follower.followPath(return3, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState  = PathState.SHOOT_4;
                    firstEnter = true;
                }
                break;

            case SHOOT_4:
                if (firstEnter) {
                    startIntakeShoot();
                    locker.setPower(lockerStates[0]);
                    stateTimer.resetTimer();
                    aimingActive = true;
                    firstEnter   = false;
                }
                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT) {
                    stopIntake();
                    locker.setPower(lockerStates[1]);
                    aimingActive = false;
                    angleServo.setPower(0);
                    pathState  = PathState.GO_TO_COLLECT_3;
                    firstEnter = true;
                }
                break;

            case GO_TO_COLLECT_3:
                if (firstEnter) {
                    startIntakeFull();
                    follower.followPath(toCollect3, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState  = PathState.COLLECT_3;
                    firstEnter = true;
                }
                break;

            case COLLECT_3:
                if (firstEnter) {
                    startIntakeFull();
                    follower.followPath(collect3, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState  = PathState.RETURN_4;
                    firstEnter = true;
                }
                break;

            case RETURN_4:
                if (firstEnter) {
                    startIntakeHold();
                    follower.followPath(return4, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState  = PathState.SHOOT_5;
                    firstEnter = true;
                }
                break;

            case SHOOT_5:
                if (firstEnter) {
                    startIntakeShoot();
                    locker.setPower(lockerStates[0]);
                    stateTimer.resetTimer();
                    aimingActive = true;
                    firstEnter   = false;
                }
                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT) {
                    stopIntake();
                    locker.setPower(lockerStates[1]);
                    aimingActive = false;
                    angleServo.setPower(0);
                    pathState  = PathState.END;
                    firstEnter = true;
                }
                break;

            case END:
                stopIntake();
                locker.setPower(lockerStates[1]);
                shootData = shooter.createCanonData(0, 0);
                angleServo.setPower(0);
                break;
        }
    }

    // ========== ПУТИ ==========
    private void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(29,133), new Pose(60,84)))
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180)).build();

        toCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60,84), new Pose(60,60)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

        collect1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60,60), new Pose(11,60)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

        return1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(11,60), new Pose(39,57), new Pose(60,84)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

        toGate = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(60,84), new Pose(50,60), new Pose(22,66.2)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

        gate = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(22,66.2), new Pose(16.5,66.2)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

        toCollect2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(16,66.2), new Pose(19,57), new Pose(9,57)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120)).build();

        return2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(9,57), new Pose(60,84)))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180)).build();

        collect2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60,84), new Pose(16,84)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

        return3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(16,84), new Pose(60,84)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

        toCollect3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60,84), new Pose(60,36)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

        collect3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60,36), new Pose(11,36)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();

        return4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(11,36), new Pose(60,84)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
    }
}