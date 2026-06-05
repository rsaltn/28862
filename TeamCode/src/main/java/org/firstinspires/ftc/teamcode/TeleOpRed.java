package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Configurable
@TeleOp(name = "TeleOp Red", group = "TeleOp")
public class TeleOpRed extends OpMode {

    Follower follower;

    CRServo locker;
    CRServo servo;          // CRServo для поворота турели

    DcMotorEx intake_l, intake_r;

    TelemetryManager telemetryP;

    public static Pose startingPose;

    Shooter shooter = new Shooter();
    Camera  cam     = new Camera();

    public double[] data;

    // ---------- НАСТРОЙКИ АВТОНАВОДКИ ДЛЯ CRServo ----------
    public static double CAMERA_TX_SIGN = 1.0;       // знак камеры (-1 если перевёрнута)
    public static double ROBOT_TURN_SIGN = 1.0;       // знак поворота робота

    public static double DEADBAND_DEG = 1;          // мёртвая зона (градусы) – не наводим
    public static double SERVO_GAIN = 0.01752;          // П-коэффициент для CRServo
    public static double SERVO_MAX_POWER = 1;       // макс. мощность CRServo

    public static double TURN_GAIN = 0.5;            // чувствительность доворота робота
    public static double MAX_AUTO_TURN = 0.15;         // макс. мощность доворота
    public static double TURN_ACTIVATE_ERROR = 2.0;    // ошибка для включения доворота (градусы)

    // Состояние
    public static boolean autoAdjacement = false;
    public static boolean lockerMode     = true;

    private boolean autoAim = true;
    private boolean llvresult = false;
    private boolean prevDpadDown = false;

    // Пресеты (RPM / угол)
    public static double PRESET_NEAR_RPM   = 100;
    public static double PRESET_NEAR_ANGLE = 0;
    public static double PRESET_MID_RPM    = 100;
    public static double PRESET_MID_ANGLE  = 0;
    public static double PRESET_FAR_RPM    = 5000;
    public static double PRESET_FAR_ANGLE  = 0;

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        data = shooter.createCanonData(0, 0);
        Constants.driveConstants.maxPower(1);
        locker.setPower(1.0);
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        Constants.driveConstants.maxPower(1);

        locker = hardwareMap.get(CRServo.class, "locker");
        servo  = hardwareMap.get(CRServo.class, "angle");

        intake_l = hardwareMap.get(DcMotorEx.class, "intake_l");
        intake_r = hardwareMap.get(DcMotorEx.class, "intake_r");
        intake_l.setDirection(DcMotorSimple.Direction.FORWARD);
        intake_r.setDirection(DcMotorSimple.Direction.FORWARD);

        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        shooter.init(hardwareMap, true);
        Camera.aprilTagPipelineIndex = 0;
        cam.init(hardwareMap);


        telemetryP = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        cam.update();
        boolean haveRecentTag = cam.hasTag;

        double driveY  = Math.cbrt(-gamepad1.left_stick_y);
        double driveX  = Math.cbrt(-gamepad1.left_stick_x);
        double driveRX = -gamepad1.right_stick_x;

        // ========== АВТОНАВОДКА ДЛЯ CRServo ==========
        if (haveRecentTag && autoAim) {
            double headingError = -cam.tx * CAMERA_TX_SIGN;   // исправленный знак

            if (Math.abs(headingError) <= DEADBAND_DEG) {
                // Уже в пределах мёртвой зоны – останавливаем серво
                servo.setPower(0);
                follower.setTeleOpDrive(driveY, driveX, driveRX, false);
            } else {
                // Пропорциональное управление CRServo
                double servoPower = headingError * SERVO_GAIN;
                servoPower = Range.clip(servoPower, -SERVO_MAX_POWER, SERVO_MAX_POWER);
                servo.setPower(servoPower);

                // Доворот роботом только если серво почти на пределе И ошибка большая
                boolean servoAtLimit = (Math.abs(servoPower) >= SERVO_MAX_POWER - 0.05);
                if (servoAtLimit && Math.abs(headingError) > TURN_ACTIVATE_ERROR) {
                    double turnPower = headingError * TURN_GAIN * ROBOT_TURN_SIGN;
                    turnPower = Range.clip(turnPower, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    follower.setTeleOpDrive(driveY, driveX, turnPower, false);
                } else {
                    follower.setTeleOpDrive(driveY, driveX, driveRX, false);
                }
            }
        } else {
            follower.setTeleOpDrive(driveY, driveX, driveRX, false);
            // Когда нет тега – останавливаем серво
            servo.setPower(0);
        }
        // =============================================

        telemetry.clearAll();
        follower.update();

        // Интейк
        double intakePower = Range.clip(gamepad1.right_trigger - gamepad1.left_trigger, -1.0, 1.0);
        intake_l.setPower(intakePower);
        intake_r.setPower(intakePower);

        // Локер
        if (gamepad1.dpad_down && !prevDpadDown) {
            lockerMode = !lockerMode;
            gamepad1.rumble(1, 1, 100);
        }
        prevDpadDown = gamepad1.dpad_down;
        locker.setPower(lockerMode ? 1.0 : 0.7);

        // Пресеты (без ручной установки серво)
        if (gamepad1.squareWasPressed()) {
            autoAdjacement = false;
            autoAim = true;
            data = shooter.createCanonData(PRESET_NEAR_RPM, PRESET_NEAR_ANGLE);
        }
        if (gamepad1.square && gamepad1.dpadLeftWasPressed()) {
            data = shooter.createCanonData(shooter.getTargetRpm() - 50, PRESET_NEAR_ANGLE);
        }
        if (gamepad1.square && gamepad1.dpadRightWasPressed()) {
            data = shooter.createCanonData(shooter.getTargetRpm() + 50, PRESET_NEAR_ANGLE);
        }

        if (gamepad1.triangleWasPressed()) {
            autoAdjacement = false;
            autoAim = true;
            data = shooter.createCanonData(PRESET_MID_RPM, PRESET_MID_ANGLE);
        }

        if (gamepad1.circleWasPressed()) {
            autoAdjacement = false;
            autoAim = true;
            data = shooter.createCanonData(PRESET_FAR_RPM, PRESET_FAR_ANGLE);
        }

        if (gamepad1.crossWasPressed()) {
            autoAdjacement = false;
            autoAim = false;
            data = shooter.createCanonData(0, 0);
            servo.setPower(0);
        }

        // Автонастройка RPM и угла по размеру апрекода
        if (gamepad1.rightBumperWasPressed()) {
            autoAdjacement = !autoAdjacement;
        }

        LLResult result = cam.camera.getLatestResult();
        llvresult = (result != null && result.isValid());
        if (llvresult && autoAdjacement) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                double x1 = fiducials.get(0).getTargetCorners().get(0).get(0);
                double y1 = fiducials.get(0).getTargetCorners().get(0).get(1);
                double x2 = fiducials.get(0).getTargetCorners().get(1).get(0);
                double y2 = fiducials.get(0).getTargetCorners().get(1).get(1);
                double x3 = fiducials.get(0).getTargetCorners().get(2).get(0);
                double y3 = fiducials.get(0).getTargetCorners().get(2).get(1);
                double x4 = fiducials.get(0).getTargetCorners().get(3).get(0);
                double y4 = fiducials.get(0).getTargetCorners().get(3).get(1);
                double corner1 = Math.hypot(x1 - x4, y1 - y4);
                double corner2 = Math.hypot(x2 - x3, y2 - y3);
                double x = (corner1 + corner2) / 2.0;
                double rpmAuto   = 0.001021838  * Math.pow(x, 3) - 0.04191788   * Math.pow(x, 2) - 27.26419 * x + 5139.389;
                double angleAuto = 0.0005790292 * Math.pow(x, 3) - 0.1392075 * Math.pow(x, 2) +   9.602636 * x - 139.1045;

                autoAim = true;
                data = shooter.createCanonData(rpmAuto, angleAuto);
                telemetryP.addData("Auto RPM",   rpmAuto);
                telemetryP.addData("Auto Angle", angleAuto);
                telemetryP.addData("Corner avg", x);
            }
        }

        shooter.shootON(data);

        if (gamepad1.share) {
            follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(0)));
            telemetry.addData("Pose reset at", getRuntime());
        }

        // Телеметрия
        telemetry.addData("Locker",             lockerMode ? "CLOSED" : "OPEN");
        telemetry.addData("AutoAdjacement",     autoAdjacement);
        telemetry.addData("AutoAim",            autoAim);
        telemetry.addData("cam.hasTag",         haveRecentTag);
        telemetry.addData("cam.tx (raw)",       cam.tx);
        telemetry.addData("Heading error",      -cam.tx * CAMERA_TX_SIGN);
        telemetry.addData("Servo power",        servo.getPower());
        telemetry.addData("Shooter target RPM", shooter.getTargetRpm());
        telemetry.addData("Shooter meas RPM",   shooter.getMeasuredRpm());
        telemetry.addData("Shooter power",      shooter.getLastPower());
        telemetry.update();
        telemetryP.update();
    }
}