package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@Configurable
@TeleOp(name = "LEDLOL", group = "TeleOp")
public class LEDLOL extends OpMode
{
    Servo led;

    public static double frequency = 0;

    public void init()
    {
        led = hardwareMap.get(Servo.class, "led");
    }
    public void loop()
    {
        frequency += 0.00001;
        frequency = frequency % 1;
        led.setPosition(frequency);
        telemetry.addData("Frequency", frequency);
    }
}