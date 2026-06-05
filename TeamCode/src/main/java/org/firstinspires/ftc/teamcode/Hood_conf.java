package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Простой OpMode для настройки сервопривода "hood" (угол наклона шутера).
 *
 * Позволяет:
 * - Менять угол через Configurable-поле panelAngle (от 0 до 360 градусов)
 * - Видеть рассчитанную позицию серво (0..1) и текущую заданную мощность/позицию
 * - Дополнительно управлять углом кнопками геймпада: D-pad Up/Down для увеличения/уменьшения угла
 *
 * Использование:
 * - Запустите OpMode "Hood Tuner" на Driver Station.
 * - Откройте панель конфигурации (Panels / Configurable), найдите поле "panelAngle".
 * - Изменяйте значение угла и наблюдайте за движением сервы.
 * - Либо используйте D-pad для плавной настройки прямо с геймпада.
 */
@Configurable
@TeleOp(name = "Hood Tuner", group = "Test")
public class Hood_conf extends OpMode {

    // Настраиваемое значение угла в градусах (0..360)
    public static double panelAngle = 0.0;

    // Серво для управления углом наклона (hood)
    private Servo hoodServo;

    // Максимальный угол (физический предел серво – можно ограничить)
    private static final double MAX_ANGLE = 360.0;
    private static final double MIN_ANGLE = 0.0;

    // Коэффициент преобразования: позиция серво = угол / 360.0
    // (при условии, что серво настроено на диапазон 0..1, где 1 = 360 градусов)

    @Override
    public void init() {
        // Получаем серво "hood" из HardwareMap (такое же имя, как в Shooter)
        hoodServo = hardwareMap.get(Servo.class, "hood");

        // Можно установить начальное положение (по желанию)
        double initialPos = angleToPosition(panelAngle);
        hoodServo.setPosition(initialPos);

        telemetry.addLine("Hood Tuner initialized");
        telemetry.addLine("Use Configurable panel to change angle (0-360°)");
        telemetry.addLine("Or use D-pad Up/Down to adjust angle by 1°");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Обработка ручного управления с геймпада (для удобства)
        if (gamepad1.dpad_up) {
            panelAngle += 1.0;
            if (panelAngle > MAX_ANGLE) panelAngle = MAX_ANGLE;
        }
        if (gamepad1.dpad_down) {
            panelAngle -= 1.0;
            if (panelAngle < MIN_ANGLE) panelAngle = MIN_ANGLE;
        }

        // 2. Преобразуем угол в позицию серво и устанавливаем
        double targetPosition = angleToPosition(panelAngle);
        hoodServo.setPosition(targetPosition);

        // 3. Отображение телеметрии
        telemetry.addData("Target Angle (deg)", panelAngle);
        telemetry.addData("Servo Position (0-1)", targetPosition);
        telemetry.addData("Hood Servo", "OK");
        telemetry.addLine("\nControl:");
        telemetry.addLine("- Change 'panelAngle' in Configurable panel");
        telemetry.addLine("- D-pad Up/Down: adjust by 1°");
        telemetry.update();
    }

    // Вспомогательный метод: преобразует угол (градусы) в позицию серво (0..1)
    private double angleToPosition(double angleDeg) {
        // Ограничиваем угол допустимыми пределами
        angleDeg = Range.clip(angleDeg, MIN_ANGLE, MAX_ANGLE);
        // Делим на 360, так как полный оборот серво обычно соответствует 1.0 (если разрешено)
        // Если у серво есть механические ограничения, можно использовать scaleRange, но здесь просто линейное отображение
        return angleDeg / 360.0;
    }
}