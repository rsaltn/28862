package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@TeleOp(name = "CHECK", group = "TeleOp")
public class CHECK extends OpMode
{
    DcMotorEx intakeF, intakeB;
    public static Pose startingPose;
    NormalizedColorSensor colorL,colorRF1, colorRR1,colorRF2, colorRR2;
    Follower follower;
    CRServo servo,servo3;
    Servo servo1,servo2, led;

    //Sorter sorter = new Sorter();

    public static double powerF = 0, powerB = 0, pos = 0,pos1 = 0.385,pos2 = 0.1,pos3 = -0.016, frequency = 0.5;

    public void init()
    {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        servo = hardwareMap.get(CRServo.class, "angle");
        servo1 = hardwareMap.get(Servo.class, "servo_left");
        servo2 = hardwareMap.get(Servo.class, "servo_right_front");
        servo3 = hardwareMap.get(CRServo.class, "servo_right_rear");

        servo2.setDirection(Servo.Direction.REVERSE);

        servo1.setDirection(Servo.Direction.REVERSE);

        led = hardwareMap.get(Servo.class, "led");

        colorL = hardwareMap.get(NormalizedColorSensor.class, "color_left");
        colorRF1 = hardwareMap.get(NormalizedColorSensor.class, "color_right_front1");
        colorRF2 = hardwareMap.get(NormalizedColorSensor.class, "color_right_front2");
        colorRR1 = hardwareMap.get(NormalizedColorSensor.class, "color_right_rear1");
        colorRR2 = hardwareMap.get(NormalizedColorSensor.class, "color_right_rear2");

        intakeF = hardwareMap.get(DcMotorEx.class, "intakeF");
        intakeB = hardwareMap.get(DcMotorEx.class, "intakeB");
        //sorter.init(hardwareMap);
    }
    public void start()
    {
        follower.startTeleopDrive();
    }

    public void loop()
    {
        follower.update();




        NormalizedRGBA rf1 = colorRF1.getNormalizedColors();
        double a_rf1 = rf1.alpha;

        double r_rf1 = 0, g_rf1 = 0, b_rf1 = 0;
        if (a_rf1 > 1e-6) {
            r_rf1 = rf1.red / a_rf1;
            g_rf1 = rf1.green / a_rf1;
            b_rf1 = rf1.blue / a_rf1;
        }
        double max_rf1 = Math.max(Math.max(r_rf1, g_rf1), b_rf1);

        telemetry.addData("RF1 A", a_rf1);
        telemetry.addData("RF1 R", r_rf1);
        telemetry.addData("RF1 G", g_rf1);
        telemetry.addData("RF1 B", b_rf1);
        telemetry.addData("RF1 Max", max_rf1);


// ====== RF2 ======
        NormalizedRGBA rf2 = colorRF2.getNormalizedColors();
        double a_rf2 = rf2.alpha;

        double r_rf2 = 0, g_rf2 = 0, b_rf2 = 0;
        if (a_rf2 > 1e-6) {
            r_rf2 = rf2.red / a_rf2;
            g_rf2 = rf2.green / a_rf2;
            b_rf2 = rf2.blue / a_rf2;
        }
        double max_rf2 = Math.max(Math.max(r_rf2, g_rf2), b_rf2);

        telemetry.addData("RF2 A", a_rf2);
        telemetry.addData("RF2 R", r_rf2);
        telemetry.addData("RF2 G", g_rf2);
        telemetry.addData("RF2 B", b_rf2);
        telemetry.addData("RF2 Max", max_rf2);


// ====== RR1 ======
        NormalizedRGBA rr1 = colorRR1.getNormalizedColors();
        double a_rr1 = rr1.alpha;

        double r_rr1 = 0, g_rr1 = 0, b_rr1 = 0;
        if (a_rr1 > 1e-6) {
            r_rr1 = rr1.red / a_rr1;
            g_rr1 = rr1.green / a_rr1;
            b_rr1 = rr1.blue / a_rr1;
        }
        double max_rr1 = Math.max(Math.max(r_rr1, g_rr1), b_rr1);

        telemetry.addData("RR1 A", a_rr1);
        telemetry.addData("RR1 R", r_rr1);
        telemetry.addData("RR1 G", g_rr1);
        telemetry.addData("RR1 B", b_rr1);
        telemetry.addData("RR1 Max", max_rr1);


// ====== RR2 ======
        NormalizedRGBA rr2 = colorRR2.getNormalizedColors();
        double a_rr2 = rr2.alpha;

        double r_rr2 = 0, g_rr2 = 0, b_rr2 = 0;
        if (a_rr2 > 1e-6) {
            r_rr2 = rr2.red / a_rr2;
            g_rr2 = rr2.green / a_rr2;
            b_rr2 = rr2.blue / a_rr2;
        }
        double max_rr2 = Math.max(Math.max(r_rr2, g_rr2), b_rr2);

        telemetry.addData("RR2 A", a_rr2);
        telemetry.addData("RR2 R", r_rr2);
        telemetry.addData("RR2 G", g_rr2);
        telemetry.addData("RR2 B", b_rr2);
        telemetry.addData("RR2 Max", max_rr2);



        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x,false);

        led.setPosition(frequency);
        servo.setPower(pos);
        servo1.setPosition(pos1);
        servo2.setPosition(pos2);

        servo3.setPower(pos3);

        intakeF.setPower(powerF);
        intakeB.setPower(powerB);
        //sorter.telemetryP.addData("Color",sorter.slotRF.getBallColor());
        //sorter.telemetryP.update();
    }
}