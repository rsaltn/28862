package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
@Disabled
@Configurable
@TeleOp(name = "TeleOp Claude ", group = "TeleOp")
public class TeleOPnew extends OpMode {

    Follower follower;

    CRServo locker;
    CRServo servo;      // "angle" — поворот турели к цели

    DcMotorEx intake_l, intake_r;

    TelemetryManager telemetryP;

    public static Pose startingPose;

    Shooter shooter = new Shooter();
    Camera  cam     = new Camera();

    public double[] data;

    // --- настройки автоприцела ---
    public static double TURN_GAIN     = 0.015;
    public static double MAX_AUTO_TURN = 0.15;
    // Коэффициент П-регулятора серво: tx в градусах, FOV Limelight ≈ ±27°
    // 1/27 ≈ 0.037 — серво на полной мощности при ошибке 27°
    // Увеличь если вялое, уменьши если дёргается
    public static double SERVO_GAIN    = 0.037;

    // --- состояние ---
    public static boolean autoAdjacement = false;
    public static boolean lockerMode     = true;  // true = закрыт

    private boolean autoAim      = true;
    private boolean llvresult    = false;
    private boolean prevDpadDown = false;

    // ── Пресеты стрельбы (@Configurable — меняются на лету) ──────────
    public static double PRESET_NEAR_RPM   = 100;
    public static double PRESET_NEAR_ANGLE = 0;
    public static double PRESET_NEAR_SERVO = -0.7;

    public static double PRESET_MID_RPM    = 100;
    public static double PRESET_MID_ANGLE  = 60;
    public static double PRESET_MID_SERVO  = -0.5;

    public static double PRESET_FAR_RPM    = 3000;
    public static double PRESET_FAR_ANGLE  = 140;
    public static double PRESET_FAR_SERVO  = -0.3;

    // ─────────────────────────────────────────────────────────────────

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        data = shooter.createCanonData(0, 0);
        Constants.driveConstants.maxPower(1);
        locker.setPower(1.0);   // стартуем закрытым
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        Constants.driveConstants.maxPower(1);

        locker = hardwareMap.get(CRServo.class, "locker");
        servo  = hardwareMap.get(CRServo.class, "angle");

        intake_l = hardwareMap.get(DcMotorEx.class, "intake_l");
        intake_r = hardwareMap.get(DcMotorEx.class, "intake_r");
        intake_l.setDirection(DcMotorSimple.Direction.REVERSE);
        intake_r.setDirection(DcMotorSimple.Direction.REVERSE);

        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        shooter.init(hardwareMap, true);
        cam.init(hardwareMap);

        telemetryP = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        cam.update();
        boolean haveRecentTag = cam.hasTag;

        // ── Движение (centric-field) + автоповорот по тегу ───────────────
        // BUG FIX: 1/3 → Math.cbrt() (целочисленное деление давало 0,
        //          Math.pow(x,0)=1 → робот всегда ехал на полной мощности)
        double driveY  = Math.cbrt(-gamepad1.left_stick_y);
        double driveX  = Math.cbrt(-gamepad1.left_stick_x);
        double driveRX = -gamepad1.right_stick_x;

        if (haveRecentTag && autoAim) {
            double headingError = -cam.tx;

            // П-регулятор: каждый луп двигаем серво пропорционально ошибке
            // tx > 0 — тег правее → серво вправо, tx < 0 — влево
            servo.setPower(Range.clip(cam.tx * SERVO_GAIN, -1.0, 1.0));

            // Если серво упёрлось в предел — доворачиваем корпусом
            if (Math.abs(servo.getPower()) >= 1.0) {
                follower.setTeleOpDrive(driveY, driveX,
                        Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN), false);
            } else {
                follower.setTeleOpDrive(driveY, driveX, driveRX, false);
            }
        } else {
            follower.setTeleOpDrive(driveY, driveX, driveRX, false);
        }

        telemetry.clearAll();
        follower.update();

        // ── Интейк / аутейк — на триггеры ────────────────────────────────
        double intakePower = Range.clip(gamepad1.right_trigger - gamepad1.left_trigger, -1.0, 1.0);
        intake_l.setPower(intakePower);
        intake_r.setPower(intakePower);

        // ── Локер — стрелка вниз, ручной фронт ───────────────────────────
        if (gamepad1.dpad_down && !prevDpadDown) {
            lockerMode = !lockerMode;
            gamepad1.rumble(1, 1, 100);
        }
        prevDpadDown = gamepad1.dpad_down;
        locker.setPower(lockerMode ? 1.0 : 0.7);

        // ── Пресеты стрельбы ──────────────────────────────────────────────
        // Square (однократно) — ближний пресет; серво ставится один раз,
        // BUG FIX: больше не перезаписывает серво каждый луп (конфликт с авто-прицелом)
        if (gamepad1.squareWasPressed()) {
            autoAdjacement = false;
            autoAim = true;
            data = shooter.createCanonData(PRESET_NEAR_RPM, PRESET_NEAR_ANGLE);
            servo.setPower(PRESET_NEAR_SERVO);
        }
        // BUG FIX: dpadLeftWasPressed/dpadRightWasPressed вместо dpad_left/dpad_right
        // (held-флаги меняли RPM ~50 раз/сек)
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
            servo.setPower(PRESET_MID_SERVO);
        }

        if (gamepad1.circleWasPressed()) {
            autoAdjacement = false;
            autoAim = true;
            data = shooter.createCanonData(PRESET_FAR_RPM, PRESET_FAR_ANGLE);
            servo.setPower(PRESET_FAR_SERVO);
        }

        if (gamepad1.crossWasPressed()) {
            autoAdjacement = false;
            autoAim = false;
            data = shooter.createCanonData(0, 0);
            servo.setPower(0);
        }

        // ── Авто-настройка RPM+угла по Limelight (right bumper вкл/выкл) ─
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
                double rpmAuto   = -0.00407296  * Math.pow(x, 3) + 1.17902   * Math.pow(x, 2) - 119.1841 * x + 7583.71974;
                double angleAuto = -0.000369038 * Math.pow(x, 3) + 0.0964091 * Math.pow(x, 2) -   9.01097 * x +  335.01112;
                autoAim = true;
                data = shooter.createCanonData(rpmAuto, angleAuto);
                telemetryP.addData("Auto RPM",   rpmAuto);
                telemetryP.addData("Auto Angle", angleAuto);
                telemetryP.addData("Corner avg", x);
            }
        }

        // ── Пушка (RPM + угол худа задаются внутри shooter.shootON) ──────
        shooter.shootON(data);

        // ── Сброс позы ───────────────────────────────────────────────────
        if (gamepad1.share) {
            follower.setPose(new Pose(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    Math.toRadians(0)));
            telemetry.addData("Pose reset at", getRuntime());
        }

        // ── Телеметрия ───────────────────────────────────────────────────
        telemetry.addData("Locker",             lockerMode ? "CLOSED" : "OPEN");
        telemetry.addData("AutoAdjacement",     autoAdjacement);
        telemetry.addData("AutoAim",            autoAim);
        telemetry.addData("cam.hasTag",         haveRecentTag);
        telemetry.addData("cam.tx",             cam.tx);
        telemetry.addData("Servo angle power",  servo.getPower());
        telemetry.addData("Shooter target RPM", shooter.getTargetRpm());
        telemetry.addData("Shooter meas RPM",   shooter.getMeasuredRpm());
        telemetry.addData("Shooter power",      shooter.getLastPower());
        telemetry.addData("intake_l",           intake_l.getPower());
        telemetry.addData("intake_r",           intake_r.getPower());
        telemetry.addData("Shooter L vel",      shooter.left_canon.getVelocity());
        telemetry.addData("Shooter R vel",      shooter.right_canon.getVelocity());
        telemetry.addData("Loop time",          getRuntime());
        telemetry.update();
        telemetryP.update();
    }
}