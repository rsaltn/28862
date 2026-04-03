package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "HoodTest2", group = "TeleOp")
public class LockerTest extends OpMode {

    Servo hood;
    CRServo locker;

    public static double posL = 0.1;


    public static double posH = 0.1;
    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class,"hood");
        locker = hardwareMap.get(CRServo.class,"locker");
    }


    @Override
    public void loop() {
        hood.setPosition(posH);
        locker.setPower(posL);

    }

}
