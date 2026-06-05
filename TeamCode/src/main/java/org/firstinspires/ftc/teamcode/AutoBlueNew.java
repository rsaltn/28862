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
@Autonomous(name = "Auto Blue ", group = "Digital")
public class AutoBlueNew extends OpMode {

    // ========== НАСТРОЙКИ ==========
    public static double[] lockerStates = {0.7, 1.0};          // открыт, закрыт
    public static double TIME_TO_SHOOT = 2.0;
    public static double TIME_FOR_GATE_INTAKE = 5.0;
    public static double[] shootRPMangle = {3750, 120};          // RPM, угол стрельбы

    // Мощность интейка
    public static double INTAKE_POWER_COLLECT = 1.0;           // полная мощность при сборе
    public static double INTAKE_POWER_HOLD = 0.25;             // слабая мощность при движении (удержание)
    public static double INTAKE_POWER_SHOOT = 1.0;             // мощность при стрельбе (подача мячей)

    // ========== ОБОРУДОВАНИЕ ==========
    private Follower follower;
    private CRServo locker;
    private DcMotorEx intake_l, intake_r;
    private Shooter shooter = new Shooter();

    // ========== АВТОМАТ СОСТОЯНИЙ ==========
    private enum PathState {
        START, SHOOT_1, GO_TO_COLLECT_1, COLLECT_1, RETURN_1,
        SHOOT_2, TO_GATE, GATE, GO_TO_COLLECT_2, RETURN_2,
        SHOOT_3, COLLECT_2, RETURN_3,
        SHOOT_4, GO_TO_COLLECT_3, COLLECT_3, RETURN_4,
        SHOOT_5, END
    }
    private PathState pathState = PathState.START;
    private Timer stateTimer = new Timer();
    private boolean firstEnter = true;
    private boolean emergencyExit = false;
    private boolean endPathBuilt = false;

    // Пути
    private PathChain startToShoot, toCollect1, collect1, return1,
            toGate, gate, toCollect2, return2,
            collect2, return3, toCollect3, collect3, return4,
            endPath;

    // Позы
    private final Pose startPose = new Pose(29.000, 133.000, Math.toRadians(143));
    private final Pose endPose   = new Pose(30, 58, Math.toRadians(90));

    // Данные стрельбы
    private double[] shootData;

    // ========== ИНИЦИАЛИЗАЦИЯ ==========
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        locker = hardwareMap.get(CRServo.class, "locker");
        intake_l = hardwareMap.get(DcMotorEx.class, "intake_l");
        intake_r = hardwareMap.get(DcMotorEx.class, "intake_r");
        intake_l.setDirection(DcMotorSimple.Direction.FORWARD);
        intake_r.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.init(hardwareMap, true);

        buildPaths();
        stateTimer.resetTimer();
    }

    @Override
    public void start() {
        shootData = shooter.createCanonData(shootRPMangle[0], shootRPMangle[1]);
        shooter.shootON(shootData);
        stateTimer.resetTimer();
        firstEnter = true;
        // Стартуем с удержанием (слабая мощность)
        setIntakePower(INTAKE_POWER_HOLD);
    }

    private void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(29,133), new Pose(60,84)))
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
                .build();

        toCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60,84), new Pose(60,60)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        collect1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60,60), new Pose(11,60)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        return1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(11,60), new Pose(39,57), new Pose(60,84)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        toGate = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60,84), new Pose(22,66.2)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        gate = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(22,66.2), new Pose(16.5,66.2)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        toCollect2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(16,66.2), new Pose(19,57), new Pose(11,54)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                .build();

        return2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(11,54), new Pose(60,84)))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .build();

        collect2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60,84), new Pose(16,84)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        return3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(16,84), new Pose(60,84)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        toCollect3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60,84), new Pose(60,36)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        collect3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60,36), new Pose(11,36)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        return4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(11,36), new Pose(60,84)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    // ========== ОСНОВНОЙ ЦИКЛ ==========
    @Override
    public void loop() {
        follower.update();
        updateFSM();
        shooter.shootON(shootData);

        telemetry.addData("State", pathState);
        telemetry.addData("Intake power", intake_l.getPower());
        telemetry.addData("Shooter target RPM", shooter.getTargetRpm());
        telemetry.addData("Shooter meas RPM", shooter.getMeasuredRpm());
        telemetry.addData("Time in state", stateTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    // ========== УПРАВЛЕНИЕ ИНТЕЙКОМ ==========
    private void setIntakePower(double power) {
        power = Range.clip(power, -1.0, 1.0);
        intake_l.setPower(power);
        intake_r.setPower(power);
    }

    private void startIntakeFull() {
        setIntakePower(INTAKE_POWER_COLLECT);
    }

    private void startIntakeHold() {
        setIntakePower(INTAKE_POWER_HOLD);
    }

    private void startIntakeShoot() {
        setIntakePower(INTAKE_POWER_SHOOT);
    }

    private void stopIntake() {
        setIntakePower(0);
    }

    // ========== КОНЕЧНЫЙ АВТОМАТ ==========
    private void updateFSM() {
        // Аварийный выход
        if (!emergencyExit && getRuntime() > 28) {
            emergencyExit = true;
            if (!endPathBuilt) {
                endPath = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), endPose))
                        .setLinearHeadingInterpolation(follower.getHeading(), endPose.getHeading())
                        .build();
                endPathBuilt = true;
            }
            follower.followPath(endPath);
            pathState = PathState.END;
        }

        switch (pathState) {
            case START:
                if (firstEnter) {
                    follower.followPath(startToShoot, true);
                    firstEnter = false;
                    stateTimer.resetTimer();
                    startIntakeHold();   // слабое удержание во время движения к первой стрельбе
                }
                if (!follower.isBusy() && stateTimer.getElapsedTimeSeconds() > 0.5) {
                    pathState = PathState.SHOOT_1;
                    firstEnter = true;
                }
                break;

            case SHOOT_1:
                if (firstEnter) {
                    startIntakeShoot();      // полная мощность для подачи мячей
                    locker.setPower(lockerStates[0]);   // открыть локер
                    stateTimer.resetTimer();
                    firstEnter = false;
                }
                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT) {
                    stopIntake();            // остановим после стрельбы
                    locker.setPower(lockerStates[1]);   // закрыть локер
                    pathState = PathState.GO_TO_COLLECT_1;
                    firstEnter = true;
                }
                break;

            case GO_TO_COLLECT_1:
                if (firstEnter) {
                    startIntakeFull();       // полная мощность для сбора по пути
                    follower.followPath(toCollect1, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState = PathState.COLLECT_1;
                    firstEnter = true;
                }
                break;

            case COLLECT_1:
                if (firstEnter) {
                    startIntakeFull();       // продолжаем собирать
                    follower.followPath(collect1, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState = PathState.RETURN_1;
                    firstEnter = true;
                }
                break;

            case RETURN_1:
                if (firstEnter) {
                    startIntakeHold();       // слабое удержание при возврате
                    follower.followPath(return1, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_2;
                    firstEnter = true;
                }
                break;

            case SHOOT_2:
                if (firstEnter) {
                    startIntakeShoot();
                    locker.setPower(lockerStates[0]);
                    stateTimer.resetTimer();
                    firstEnter = false;
                }
                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT) {
                    stopIntake();
                    locker.setPower(lockerStates[1]);
                    pathState = PathState.TO_GATE;
                    firstEnter = true;
                }
                break;

            case TO_GATE:
                if (firstEnter) {
                    startIntakeHold();       // слабое удержание
                    follower.followPath(toGate, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState = PathState.GATE;
                    firstEnter = true;
                }
                break;

            case GATE:
                if (firstEnter) {
                    follower.followPath(gate, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState = PathState.GO_TO_COLLECT_2;
                    firstEnter = true;
                }
                break;

            case GO_TO_COLLECT_2:
                if (firstEnter) {
                    startIntakeFull();
                    follower.followPath(toCollect2, true);
                    firstEnter = false;
                    stateTimer.resetTimer();
                }
                if (!follower.isBusy()) {
                    pathState = PathState.RETURN_2;
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
                    pathState = PathState.SHOOT_3;
                    firstEnter = true;
                }
                break;

            case SHOOT_3:
                if (firstEnter) {
                    startIntakeShoot();
                    locker.setPower(lockerStates[0]);
                    stateTimer.resetTimer();
                    firstEnter = false;
                }
                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT) {
                    stopIntake();
                    locker.setPower(lockerStates[1]);
                    pathState = PathState.COLLECT_2;
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
                    pathState = PathState.RETURN_3;
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
                    pathState = PathState.SHOOT_4;
                    firstEnter = true;
                }
                break;

            case SHOOT_4:
                if (firstEnter) {
                    startIntakeShoot();
                    locker.setPower(lockerStates[0]);
                    stateTimer.resetTimer();
                    firstEnter = false;
                }
                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT) {
                    stopIntake();
                    locker.setPower(lockerStates[1]);
                    pathState = PathState.GO_TO_COLLECT_3;
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
                    pathState = PathState.COLLECT_3;
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
                    pathState = PathState.RETURN_4;
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
                    pathState = PathState.SHOOT_5;
                    firstEnter = true;
                }
                break;

            case SHOOT_5:
                if (firstEnter) {
                    startIntakeShoot();
                    locker.setPower(lockerStates[0]);
                    stateTimer.resetTimer();
                    firstEnter = false;
                }
                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT) {
                    stopIntake();
                    locker.setPower(lockerStates[1]);
                    pathState = PathState.END;
                    firstEnter = true;
                }
                break;

            case END:
                stopIntake();
                locker.setPower(lockerStates[1]);
                shootData = shooter.createCanonData(0, 0);
                shooter.shootON(shootData);
                break;
        }
    }
}