package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@Configurable
@TeleOp(name = "Angle Servo Test", group = "Test")
public class Shooter_angle_test extends OpMode {

    CRServo servo;

    public static double targetPower = 0.0;

    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class, "angle");
        servo.setPower(0);
    }

    @Override
    public void loop() {
        double power = Range.clip(targetPower, -1.0, 1.0);
        servo.setPower(power);

        // Можно также рулить правым стиком геймпада для быстрой проверки
        // (right_stick_y переопределяет targetPower пока зажат)
        if (Math.abs(gamepad1.right_stick_y) > 0.05) {
            power = -gamepad1.right_stick_y;
            servo.setPower(power);
        }

        telemetry.addData("targetPower (Configurable)", targetPower);
        telemetry.addData("actual setPower",            power);
        telemetry.addData("gamepad right_stick_y",      gamepad1.right_stick_y);
        telemetry.addLine("--- Управление ---");
        telemetry.addLine("Configurables: меняй targetPower");
        telemetry.addLine("Геймпад: right_stick_y — прямое управление");
        telemetry.update();
    }

    @Override
    public void stop() {
        servo.setPower(0);
    }
}