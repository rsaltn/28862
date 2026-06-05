package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@Configurable
@TeleOp(name = "Calibration", group = "Utils")
public class CalibrationOpMode extends OpMode {
    DcMotorEx intake_l, intake_r;

    Camera  cam     = new Camera();
    Shooter shooter = new Shooter();

    public static int    pipelineIndex = 1;
    public static double RPM_STEP      = 50;
    public static double ANGLE_STEP    = 1.0;

    private double targetRpm   = 1000;
    private double targetAngle = 0;
    private double[] data;

    private boolean prevUp    = false;
    private boolean prevDown  = false;
    private boolean prevLeft  = false;
    private boolean prevRight = false;
    private boolean prevSave  = false;

    private final StringBuilder log = new StringBuilder();
    private int pointCount = 0;

    @Override
    public void init() {
        intake_l = hardwareMap.get(DcMotorEx.class, "intake_l");
        intake_r = hardwareMap.get(DcMotorEx.class, "intake_r");
        Camera.aprilTagPipelineIndex = pipelineIndex;
        cam.init(hardwareMap);
        shooter.init(hardwareMap, true);
        data = shooter.createCanonData(0, 0);
        telemetry.addLine("↑↓ RPM  ←→ Angle  X=Save  O=Stop");
        telemetry.update();
    }

    @Override
    public void loop() {
        cam.update();

        // Управление RPM
        if (gamepad1.dpad_up && !prevUp)    { targetRpm += RPM_STEP;   gamepad1.rumble(0.3, 0, 80); }
        if (gamepad1.dpad_down && !prevDown) { targetRpm -= RPM_STEP;   gamepad1.rumble(0, 0.3, 80); }
        prevUp   = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;

        // Управление углом
        if (gamepad1.dpad_right && !prevRight) { targetAngle += ANGLE_STEP; gamepad1.rumble(0.3, 0, 80); }
        if (gamepad1.dpad_left  && !prevLeft)  { targetAngle -= ANGLE_STEP; gamepad1.rumble(0, 0.3, 80); }
        prevRight = gamepad1.dpad_right;
        prevLeft  = gamepad1.dpad_left;

        double intakePower = Range.clip(gamepad1.right_trigger - gamepad1.left_trigger, -1.0, 1.0);
        intake_l.setPower(intakePower);
        intake_r.setPower(intakePower);

        // Стоп
        if (gamepad1.circle) { targetRpm = 0; targetAngle = 0; }

        data = shooter.createCanonData(targetRpm, targetAngle);
        shooter.shootON(data);

        // Данные камеры
        double cornerAvg = 0;
        double tagTx = 0, tagTy = 0, tagTa = 0;
        boolean tagVisible = false;

        LLResult result = cam.camera.getLatestResult();
        if (result != null && result.isValid()) {
            tagTx = result.getTx();
            tagTy = result.getTy();
            tagTa = result.getTa();

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                tagVisible = true;
                LLResultTypes.FiducialResult f = fiducials.get(0);

                double x1 = f.getTargetCorners().get(0).get(0);
                double y1 = f.getTargetCorners().get(0).get(1);
                double x2 = f.getTargetCorners().get(1).get(0);
                double y2 = f.getTargetCorners().get(1).get(1);
                double x3 = f.getTargetCorners().get(2).get(0);
                double y3 = f.getTargetCorners().get(2).get(1);
                double x4 = f.getTargetCorners().get(3).get(0);
                double y4 = f.getTargetCorners().get(3).get(1);
                cornerAvg = (Math.hypot(x1-x4, y1-y4) + Math.hypot(x2-x3, y2-y3)) / 2.0;
            }
        }

        // Сохранить точку
        if (gamepad1.cross && !prevSave && tagVisible) {
            pointCount++;
            log.append(String.format(
                    "#%d | x=%.1f | RPM=%.0f | Angle=%.1f\n",
                    pointCount, cornerAvg, targetRpm, targetAngle
            ));
            gamepad1.rumble(1, 1, 200);
        }
        prevSave = gamepad1.cross;

        // Телеметрия
        telemetry.clearAll();
        telemetry.addData("Тег",          tagVisible ? "ДА ✓" : "НЕТ");
        telemetry.addData("Corner avg x", String.format("%.2f px", cornerAvg));
        telemetry.addData("TX / TY",      String.format("%.2f° / %.2f°", tagTx, tagTy));
        telemetry.addLine("");
        telemetry.addData("Target RPM",   targetRpm);
        telemetry.addData("Measured RPM", shooter.getMeasuredRpm());
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addLine("");
        telemetry.addLine("↑↓ RPM  ←→ Angle  X=Save  O=Stop");

        if (pointCount > 0) {
            telemetry.addLine("--- Лог ---");
            telemetry.addLine(log.toString());
        }

        telemetry.update();
    }
}