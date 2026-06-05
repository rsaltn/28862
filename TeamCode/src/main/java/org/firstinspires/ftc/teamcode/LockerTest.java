package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Locker+ hood cr loop", group = "TeleOp")
public class LockerTest extends OpMode {

    CRServo hood;
    CRServo locker;

    CRServo angle;

    public static double posL = 1;


    public static double posH = 0.1;
    public static double posAngle = 0;
    @Override
    public void init() {
        hood = hardwareMap.get(CRServo.class,"hood");
        locker = hardwareMap.get(CRServo.class,"locker");
        angle = hardwareMap.get(CRServo.class,"angle");
    }
//------------------------------------
// TODO: hood range from -0.3  to 1
//------------------------------------
    @Override
    public void loop() {
        hood.setPower(posH);
        locker.setPower(posL);
        angle.setPower(posAngle);

    }

}
