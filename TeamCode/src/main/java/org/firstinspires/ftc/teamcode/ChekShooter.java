package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * OpMode для управления только шутером через панель.
 *
 * Использование:
 * - Откройте панель конфигурации (Panels / Configurable) — там будут доступны
 *   поля panelRpm, panelAngle и spinShooter.
 * - Изменяя panelRpm и panelAngle, вы задаёте целевые параметры шутера.
 * - Переключатель spinShooter включает/выключает мотор шутера (вращение на заданной скорости).
 *
 * Этот файл основан на вашем TeleOP и инициализирует только шутер; драйвинг,
 * и другая логика из оригинального TeleOP не используются здесь.
 */
@Configurable
@TeleOp(name = "Shooter Panel Control", group = "TeleOp")
public class ChekShooter extends OpMode {

    TelemetryManager telemetryP;
    Shooter shooter = new Shooter();

    // Эти поля можно изменять из панели (Configurable)
    public static double panelRpm = 10000.0;    // целевая скорость в RPM
    public static double panelAngle = 0.0;     // угол (если у вас используется)
    public static boolean spinShooter = true; // включить/выключить вращение шутера

    private double[] data;

    @Override
    public void init() {
        // Инициализируем только шутер
        shooter.init(hardwareMap, true);

        // Получаем интерфейс панели телеметрии
        telemetryP = PanelsTelemetry.INSTANCE.getTelemetry();

        // Инициализируем данные шутера по значениям из панели
        data = shooter.createCanonData(panelRpm, panelAngle);
        shooter.shootON(data); // попытка установить начальные параметры (скорость 0/значение в зависимости от spinShooter в start)
    }

    @Override
    public void start() {
        // При старте применяем текущие параметры
        data = shooter.createCanonData(panelRpm, panelAngle);
        if (spinShooter) {
            shooter.shootON(data);
        } else {
            shooter.shootON(shooter.createCanonData(0, panelAngle));
        }
        shooter.shootON(data);
    }

    @Override
    public void loop() {
        // Обновляем данные шутера на основе значений с панели
        data = shooter.createCanonData(panelRpm, panelAngle);

        if (spinShooter) {
            shooter.shootON(data);
        } else {
            // если выключено — подаём нулевую скорость, но оставляем угол
            shooter.shootON(shooter.createCanonData(0, panelAngle));
        }

        // Отображаем значения в панели и стандартной телеметрии
        telemetryP.addData("PanelRPM", panelRpm);
        telemetryP.addData("PanelAngle", panelAngle);
        telemetryP.addData("SpinShooter", spinShooter);

        // Информация от класса Shooter (если реализовано в вашем проекте)
        try {
            telemetryP.addData("Shooter target RPM", shooter.getTargetRpm());
            telemetryP.addData("Shooter meas RPM", shooter.getMeasuredRpm());
            telemetryP.addData("Shooter power", shooter.getLastPower());
        } catch (Exception ignored) {
            // Если методы отсутствуют или бросают исключение — просто игнорируем
        }
        if (gamepad1.aWasPressed()){
            spinShooter = true;
        }
        if (gamepad1.bWasPressed()){
            spinShooter = false;
        }

        // Обновляем отображение
        telemetry.update();
        telemetryP.update();
    }

    @Override
    public void stop() {
        // Отключаем шутер при остановке
        shooter.shootON(shooter.createCanonData(0, panelAngle));
    }
}
