package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Shooter {

    public DcMotorEx left_canon, right_canon;

    public Servo hoodAngle;

    public static double basic_velocity_rpm = 4000;
    public static double minVelocity_rpm = 300;
    public static double maxVelocity_rpm = 6000;

    public static double basic_angle = 0;
    public static double excessiveVelocityTolerance_rpm = 300;

    public static double TBE_CPR = 8192.0;

    public static double kP = 0.0006;
    public static double kI = 0.000000;
    public static double kD = 0.0;

    public static double kF = 0.00022;

    public static double maxPower = 1.0;

    private final ElapsedTime timer = new ElapsedTime();
    private int lastTicks = 0;
    private double lastTime = 0.0;

    private double measuredRpm = 0.0;
    private double targetRpm = 0.0;

    private double integral = 0.0;
    private double lastError = 0.0;

    private double lastPower = 0.0;

    public double angleToPosition(double angle) {
        return (angle / 360.0);
    }

    public double[] createCanonData(double velocityRpm, double angle) {
        return new double[]{velocityRpm, angle};
    }

    public void init(HardwareMap hardwareMap, boolean reverse) {

        left_canon  = hardwareMap.get(DcMotorEx.class, "shooter_l");
        right_canon = hardwareMap.get(DcMotorEx.class, "shooter_r");

        if (reverse) {
            right_canon.setDirection(DcMotorSimple.Direction.REVERSE);
            left_canon.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            right_canon.setDirection(DcMotorSimple.Direction.FORWARD);
            left_canon.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        left_canon.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_canon.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_canon.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_canon.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_canon.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lastTicks = (right_canon.getCurrentPosition() + left_canon.getCurrentPosition())/2;
        lastTime = timer.seconds();

        integral = 0.0;
        lastError = 0.0;
        lastPower = 0.0;

        hoodAngle = hardwareMap.get(Servo.class, "hood");
        hoodAngle.setDirection(Servo.Direction.REVERSE);
        hoodAngle.scaleRange(0,0.43);

    }

    private void updateMeasuredRpm() {
        int ticks = (right_canon.getCurrentPosition() + left_canon.getCurrentPosition())/2;
        double t = timer.seconds();
        double dt = t - lastTime;

        if (dt > 1e-3) {
            double ticksPerSec = (ticks - lastTicks) / dt;

            ticksPerSec = Math.abs(ticksPerSec);

            measuredRpm = (ticksPerSec / TBE_CPR) * 60.0;

            lastTicks = ticks;
            lastTime = t;
        }
    }

    private double computePower(double targetRpm) {
        double error = targetRpm - measuredRpm;

        double t = timer.seconds();
        double dt = t - lastTime;
        if (dt < 1e-3) dt = 1e-3;

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = (kF * targetRpm) + (kP * error) + (kI * integral) + (kD * derivative);

        power = Range.clip(power, 0.0, maxPower);

        lastPower = power;
        return power;
    }

    public void shootON(double[] data) {
        updateMeasuredRpm();

        double velocityRpm = data[0];
        double angle = data[1];

        if ((velocityRpm > (maxVelocity_rpm + excessiveVelocityTolerance_rpm)) ||
                (velocityRpm < (minVelocity_rpm - excessiveVelocityTolerance_rpm))) {
            targetRpm = basic_velocity_rpm;
            hoodAngle.setPosition(angleToPosition(basic_angle));
        } else {
            targetRpm = velocityRpm;
            hoodAngle.setPosition(angleToPosition(angle));
        }

        if (targetRpm <= 1) {
            left_canon.setPower(0);
            right_canon.setPower(0);
            integral = 0;
            lastPower = 0;
            return;
        }

        double power = computePower(targetRpm);

        left_canon.setPower(power);
        right_canon.setPower(power);
    }

    public double getMeasuredRpm() { return measuredRpm; }
    public double getTargetRpm() { return targetRpm; }
    public double getLastPower() { return lastPower; }
}
