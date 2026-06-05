package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Configurable
@TeleOp(name = "config", group = "Linear OpMode")
public class Config extends LinearOpMode {

    public static double DRIVE_POWER = 0.5;
    public static double SHOOTER_POWER = 1.0;
    public static double INTAKE_POWER = 1.0;
    public static double SERVO_STEP = 0.01;

    public static double HOOD_START = 0.50;
    public static double LOCKER_START = 0.50;

    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx lf, rf, lr, rr;
    private DcMotorEx intake_l, intake_r;
    private DcMotorEx shooter_l, shooter_r;

//    private Servo hood;
//    private Servo locker;

    private double hoodPos;
    private double lockerPos;

    @Override
    public void runOpMode() {

        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        intake_l = hardwareMap.get(DcMotorEx.class, "intake_l");
        intake_r = hardwareMap.get(DcMotorEx.class, "intake_r");

        shooter_l = hardwareMap.get(DcMotorEx.class, "shooter_l");
        shooter_r = hardwareMap.get(DcMotorEx.class, "shooter_r");

//        hood = hardwareMap.get(Servo.class, "hood");
//        locker = hardwareMap.get(Servo.class, "locker");

        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lr.setDirection(DcMotor.Direction.FORWARD);
        rr.setDirection(DcMotor.Direction.FORWARD);

        intake_l.setDirection(DcMotor.Direction.REVERSE);
        intake_r.setDirection(DcMotor.Direction.REVERSE);

        shooter_l.setDirection(DcMotor.Direction.REVERSE);
        shooter_r.setDirection(DcMotor.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter_l.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter_r.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        hoodPos = HOOD_START;
//        lockerPos = LOCKER_START;

//        hood.setPosition(hoodPos);
//        locker.setPosition(lockerPos);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            stopAllMotors();

            if (gamepad1.y) {
                lf.setPower(DRIVE_POWER);
            }
            if (gamepad1.x) {
                rf.setPower(DRIVE_POWER);
            }
            if (gamepad1.a) {
                rr.setPower(DRIVE_POWER);
            }
            if (gamepad1.b) {
                lr.setPower(DRIVE_POWER);
            }

            if (gamepad1.left_bumper) {
                shooter_l.setPower(SHOOTER_POWER);
            }
            if (gamepad1.right_bumper) {
                shooter_r.setPower(SHOOTER_POWER);
            }

            if (gamepad1.right_trigger > 0.1) {
                intake_l.setPower(INTAKE_POWER);
            }
            if (gamepad1.left_trigger > 0.1) {
                intake_r.setPower(INTAKE_POWER);
            }

            if (gamepad1.dpad_up) {
                hoodPos += SERVO_STEP;
            }
            if (gamepad1.dpad_down) {
                hoodPos -= SERVO_STEP;
            }
            if (gamepad1.dpad_right) {
                lockerPos += SERVO_STEP;
            }
            if (gamepad1.dpad_left) {
                lockerPos -= SERVO_STEP;
            }

            if (hoodPos < 0) hoodPos = 0;
            if (hoodPos > 1) hoodPos = 1;

            if (lockerPos < 0) lockerPos = 0;
            if (lockerPos > 1) lockerPos = 1;

//            hood.setPosition(hoodPos);
//            locker.setPosition(lockerPos);

            telemetry.addData("Runtime", runtime.seconds());
            telemetry.addData("lf", lf.getPower());
            telemetry.addData("rf", rf.getPower());
            telemetry.addData("lr", lr.getPower());
            telemetry.addData("rr", rr.getPower());
            telemetry.addData("shooter_l", shooter_l.getPower());
            telemetry.addData("shooter_r", shooter_r.getPower());
            telemetry.addData("intake_l", intake_l.getPower());
            telemetry.addData("intake_r", intake_r.getPower());
            telemetry.addData("hood", hoodPos);
            telemetry.addData("locker", lockerPos);
            telemetry.update();
        }

        stopAllMotors();
    }

    private void stopAllMotors() {
        lf.setPower(0);
        rf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);
        intake_l.setPower(0);
        intake_r.setPower(0);
        shooter_l.setPower(0);
        shooter_r.setPower(0);
    }
}