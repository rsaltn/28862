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
@Autonomous(name = "Auto Blue Far", group = "Digital")
public class AutoBlueFar extends OpMode {

    // ========== НАСТРОЙКИ ==========
    public static double[] lockerStates = {0.7, 1.0};      // открыт/закрыт
    public static double TIME_TO_SHOOT_FIRST = 4.5;        // время первого выстрела
    public static double TIME_TO_SHOOT_SHORT = 2.0;        // время последующих выстрелов
    public static double[] shootRPMangle = {4800, 160};    // RPM, угол для дальней зоны
    public static double RPM_THRESHOLD = 4600;              // мин. RPM для включения интейка

    public static double INTAKE_POWER_SHOOT = 1.0;          // мощность интейка при стрельбе
    public static double INTAKE_POWER_HOLD = 0.7;
    public static double INTAKE_POWER_COLLECT = 1.0;

    public static double TOTAL_CYCLE_TIME = 29.5;           // общее время работы циклов
    public static double[] FINAL_POSE = {31, 17, 180};      // конечная позиция (x, y, heading в градусах)

    // Настройки камеры (автонаводка)
    public static double CAMERA_TX_SIGN = 1.0;
    public static double DEADBAND_DEG = 1.2;
    public static double SERVO_GAIN = 0.025;
    public static double SERVO_MAX_POWER = 0.7;

    // ========== ОБОРУДОВАНИЕ ==========
    private Follower follower;
    private CRServo locker;
    private CRServo angleServo;
    private DcMotorEx intake_l, intake_r;
    private Shooter shooter = new Shooter();
    private Camera  cam     = new Camera();

    // ========== АВТОМАТ СОСТОЯНИЙ ==========
    private enum PathState {
        SHOOT_FIRST,
        LONG_GO,
        LONG_COLLECT,
        LONG_RETURN,
        SHOOT_SHORT,
        SHORT_GO,
        SHORT_RETURN,
        FINAL_DRIVE,
        END
    }
    private PathState pathState = PathState.SHOOT_FIRST;
    private Timer stateTimer = new Timer();
    private boolean firstEnter = true;

    // Пути
    private PathChain long_to_collect;
    private PathChain long_collect;
    private PathChain long_return;
    private PathChain short_go;
    private PathChain short_return;

    // Позы
    private final Pose shootPose = new Pose(59, 7, Math.toRadians(180));   // теперь это и стартовая позиция
    private final Pose collectShortPose = new Pose(10, 7, Math.toRadians(180));
    private final Pose finalPose = new Pose(FINAL_POSE[0], FINAL_POSE[1], Math.toRadians(FINAL_POSE[2]));

    private double[] shootData;
    private boolean aimingActive = false;

    // Глобальный таймер автономки
    private double autoStartTime;

    // ========== ИНИЦИАЛИЗАЦИЯ ==========
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(shootPose);   // стартуем сразу на позиции стрельбы

        locker = hardwareMap.get(CRServo.class, "locker");
        angleServo = hardwareMap.get(CRServo.class, "angle");
        intake_l = hardwareMap.get(DcMotorEx.class, "intake_l");
        intake_r = hardwareMap.get(DcMotorEx.class, "intake_r");

        shooter.init(hardwareMap, true);
        cam.init(hardwareMap);

        buildPaths();

        locker.setPower(lockerStates[1]); // закрыт
        setIntakePower(0);
        angleServo.setPower(0);
    }

    @Override
    public void start() {
        shootData = shooter.createCanonData(shootRPMangle[0], shootRPMangle[1]);
        shooter.shootON(shootData);
        stateTimer.resetTimer();
        firstEnter = true;
        pathState = PathState.SHOOT_FIRST;
        autoStartTime = getRuntime();
    }

    private void buildPaths() {
        // Длинный путь на сбор (через дальнюю зону)
        long_to_collect = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(30, 17), new Pose(10, 24)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(270))
                .build();

        long_collect = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(10, 24), new Pose(10, 12)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .build();

        long_return = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(10, 12), shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(270), shootPose.getHeading())
                .build();

        // Короткие пути: сразу к точке сбора и обратно
        short_go = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collectShortPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collectShortPose.getHeading())
                .build();

        short_return = follower.pathBuilder()
                .addPath(new BezierLine(collectShortPose, shootPose))
                .setLinearHeadingInterpolation(collectShortPose.getHeading(), shootPose.getHeading())
                .build();
    }

    @Override
    public void loop() {
        cam.update();
        follower.update();
        updateFSM();

        if (aimingActive) {
            performAiming();
        }

        shooter.shootON(shootData);

        telemetry.addData("State", pathState);
        telemetry.addData("Time", getRuntime() - autoStartTime);
        telemetry.addData("Shooter RPM", shooter.getMeasuredRpm());
        telemetry.addData("Target RPM", shooter.getTargetRpm());
        telemetry.addData("Intake power", intake_l.getPower());
        telemetry.update();
    }

    private void performAiming() {
        if (!cam.hasTag) return;
        double headingError = -cam.tx * CAMERA_TX_SIGN;
        if (Math.abs(headingError) <= DEADBAND_DEG) {
            angleServo.setPower(0);
        } else {
            angleServo.setPower(Range.clip(headingError * SERVO_GAIN, -SERVO_MAX_POWER, SERVO_MAX_POWER));
        }
    }

    private void setIntakePower(double power) {
        power = Range.clip(power, -1.0, 1.0);
        intake_l.setPower(power);
        intake_r.setPower(power);
    }

    private boolean isTimeOver() {
        return (getRuntime() - autoStartTime) >= TOTAL_CYCLE_TIME;
    }

    private void updateFSM() {
        switch (pathState) {
            case SHOOT_FIRST:
                if (firstEnter) {
                    aimingActive = true;
                    locker.setPower(lockerStates[0]);
                    stateTimer.resetTimer();
                    firstEnter = false;
                }

                if (shooter.getMeasuredRpm() > RPM_THRESHOLD) {
                    setIntakePower(INTAKE_POWER_SHOOT);
                } else {
                    setIntakePower(0);
                }

                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT_FIRST) {
                    setIntakePower(0);
                    locker.setPower(lockerStates[1]);
                    aimingActive = false;
                    angleServo.setPower(0);
                    pathState = PathState.LONG_GO;
                    firstEnter = true;
                }
                break;

            case LONG_GO:
                if (firstEnter) {
                    follower.followPath(long_to_collect, true);
                    firstEnter = false;
                    setIntakePower(INTAKE_POWER_COLLECT);
                }
                if (!follower.isBusy()) {
                    pathState = PathState.LONG_COLLECT;
                    firstEnter = true;
                }
                break;

            case LONG_COLLECT:
                if (firstEnter) {
                    follower.followPath(long_collect, true);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState = PathState.LONG_RETURN;
                    firstEnter = true;
                }
                break;

            case LONG_RETURN:
                if (firstEnter) {
                    follower.followPath(long_return, true);
                    firstEnter = false;
                    setIntakePower(INTAKE_POWER_HOLD);
                }
                if (!follower.isBusy()) {
                    if (isTimeOver()) {
                        pathState = PathState.FINAL_DRIVE;
                    } else {
                        pathState = PathState.SHOOT_SHORT;
                    }
                    firstEnter = true;
                }
                break;

            case SHOOT_SHORT:
                if (firstEnter) {
                    aimingActive = true;
                    locker.setPower(lockerStates[0]);
                    stateTimer.resetTimer();
                    firstEnter = false;
                }

                if (shooter.getMeasuredRpm() > RPM_THRESHOLD) {
                    setIntakePower(INTAKE_POWER_SHOOT);
                } else {
                    setIntakePower(0);
                }

                if (stateTimer.getElapsedTimeSeconds() > TIME_TO_SHOOT_SHORT) {
                    setIntakePower(0);
                    locker.setPower(lockerStates[1]);
                    aimingActive = false;
                    angleServo.setPower(0);

                    if (isTimeOver()) {
                        pathState = PathState.FINAL_DRIVE;
                    } else {
                        pathState = PathState.SHORT_GO;
                    }
                    firstEnter = true;
                }
                break;

            case SHORT_GO:
                if (firstEnter) {
                    follower.followPath(short_go, true);
                    firstEnter = false;
                    setIntakePower(INTAKE_POWER_COLLECT);
                }
                if (!follower.isBusy()) {
                    pathState = PathState.SHORT_RETURN;
                    firstEnter = true;
                }
                break;

            case SHORT_RETURN:
                if (firstEnter) {
                    follower.followPath(short_return, true);
                    firstEnter = false;
                    setIntakePower(INTAKE_POWER_HOLD);
                }
                if (!follower.isBusy()) {
                    if (isTimeOver()) {
                        pathState = PathState.FINAL_DRIVE;
                    } else {
                        pathState = PathState.SHOOT_SHORT;
                    }
                    firstEnter = true;
                }
                break;

            case FINAL_DRIVE:
                if (firstEnter) {
                    Pose currentPose = follower.getPose();
                    PathChain finalPath = follower.pathBuilder()
                            .addPath(new BezierLine(currentPose, finalPose))
                            .setLinearHeadingInterpolation(currentPose.getHeading(), finalPose.getHeading())
                            .build();
                    follower.followPath(finalPath, true);
                    setIntakePower(0);
                    firstEnter = false;
                }
                if (!follower.isBusy()) {
                    pathState = PathState.END;
                    firstEnter = true;
                }
                break;

            case END:
                setIntakePower(0);
                break;
        }
    }
}