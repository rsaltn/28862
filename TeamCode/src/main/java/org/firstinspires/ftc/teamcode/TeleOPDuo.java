package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.util.Timer;

import java.util.List;


@Configurable
@TeleOp(name = "TeleOpDuo", group = "TeleOp")
public class TeleOPDuo extends OpMode {

    Follower follower;
    Servo led;
    CRServo servo;

    DcMotorEx intakeF, intakeB, rf, rr, lf, lr;
    AnalogInput encoderT;

    TelemetryManager telemetryP;


    public static Pose startingPose;

    Shooter shooter = new Shooter();
    Sorter sorter = new Sorter();

    public double[] data;
    public static double velocity = 0, angle = 0, delayT = 0.15, constantT = -2.58, sp = 0.07,si = 0.0001,sd = 0.0006;
    int balls = 0;

    enum Pattern {
        GPP,
        PGP,
        PPG,
        NONE
    }

    public static boolean autoAdjacement = false;

    Pattern pattern = Pattern.GPP;

    Camera cam = new Camera();

    public static double tagHoldSeconds = 0.15;

    private double lastSeenTime = -999;
    public static double TURN_GAIN = 0.015;
    public static double MAX_AUTO_TURN = 0.15 ;

    Timer serv = new Timer();
    Timer move = new Timer();
    private boolean autoAim = true, delay = false, breakM = false, llvresult = true;

    // Переменные для отслеживания состояния выстрела
    private boolean isShootingInProgress = false; // чтобы предотвратить многократные выстрелы
    private int ballsShot = 0; // Счётчик выстреленных мячей

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        Constants.driveConstants.maxPower(1);

        servo = hardwareMap.get(CRServo.class, "angle");

        rf = hardwareMap.get(DcMotorEx.class, "rr");
        rr = hardwareMap.get(DcMotorEx.class, "rf");
        lf = hardwareMap.get(DcMotorEx.class, "lr");
        lr = hardwareMap.get(DcMotorEx.class, "lf");

        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        led = hardwareMap.get(Servo.class, "led");

        cam.init(hardwareMap);

        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        shooter.init(hardwareMap, true);
        sorter.init(hardwareMap);

        intakeF = hardwareMap.get(DcMotorEx.class, "intakeF");
        intakeB = hardwareMap.get(DcMotorEx.class, "intakeB");
        intakeB.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeF.setDirection(DcMotorSimple.Direction.REVERSE
        );

        telemetryP = PanelsTelemetry.INSTANCE.getTelemetry();

        encoderT = hardwareMap.get(AnalogInput.class, "encoderT");

    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
        data = shooter.createCanonData(velocity, angle);
        Constants.driveConstants.maxPower(1);

    }

    @Override
    public void loop() {

        double now = getRuntime();
        cam.update();

        boolean haveRecentTag = cam.hasTag;
        if (haveRecentTag && autoAim) {
            double headingError = -cam.tx;
            if (!delay) {
                serv.resetTimer();
                servo.setPower(servo.getPower() + cam.tx / 60);
                delay = true;
            }
            if (serv.getElapsedTimeSeconds() > delayT) {
                delay = false;
            }
            if (Math.abs(servo.getPower()) == 1 && !delay) {
                follower.setTeleOpDrive(-gamepad2.left_stick_y, -gamepad2.left_stick_x, Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN), true);
            } else {
                follower.setTeleOpDrive(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x, true);
            }
        }
        else {
            follower.setTeleOpDrive(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x, true);
        }




        telemetry.clearAll();

        follower.update();

         if (gamepad1.dpad_right  && !isShootingInProgress) {
            isShootingInProgress = true;
            ballsShot = 0;

            new Thread(() -> {
                switch (pattern) {
                    case GPP:
                        shootOneBall(Sorter.detectedColor.GREEN);
                        shootOneBall(Sorter.detectedColor.PURPLE);
                        shootOneBall(Sorter.detectedColor.PURPLE);
                        break;
                    case PGP:
                        shootOneBall(Sorter.detectedColor.PURPLE);
                        shootOneBall(Sorter.detectedColor.GREEN);
                        shootOneBall(Sorter.detectedColor.PURPLE);
                        break;
                    case PPG:
                        shootOneBall(Sorter.detectedColor.PURPLE);
                        shootOneBall(Sorter.detectedColor.PURPLE);
                        shootOneBall(Sorter.detectedColor.GREEN);
                        break;
                    default:
                        break;
                }
            }).start();
        }
        else if (gamepad1.circle)
        {
            new Thread(() -> {
                switch (pattern) {
                    case GPP:
                        if(ballsShot==1){shootOneBall(Sorter.detectedColor.PURPLE);}
                        if(ballsShot==2){shootOneBall(Sorter.detectedColor.PURPLE);}
                        if(ballsShot>2){shootOneBall(Sorter.detectedColor.PURPLE);}

                        break;
                    case PGP:
                        if(ballsShot==1){shootOneBall(Sorter.detectedColor.GREEN);}
                        if(ballsShot==2){shootOneBall(Sorter.detectedColor.PURPLE);}
                        if(ballsShot>2){shootOneBall(Sorter.detectedColor.PURPLE);}

                        break;
                    case PPG:
                        if(ballsShot==1){shootOneBall(Sorter.detectedColor.PURPLE);}
                        if(ballsShot==2){shootOneBall(Sorter.detectedColor.GREEN);}
                        if(ballsShot>2){shootOneBall(Sorter.detectedColor.PURPLE);}

                        break;
                    default:
                        break;
                }
            }).start();
        }
        if(!gamepad1.circle){isShootingInProgress = false;ballsShot = 0;}




        if (lr.getPower() > 0.01 && lf.getPower() > 0.01 && rf.getPower() > 0.01 && rr.getPower() > 0.01) {
            intakeF.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            intakeB.setPower(0);
        } else if (lr.getPower() < -0.01 && lf.getPower() < -0.01 && rf.getPower() < -0.01 && rr.getPower() < -0.01) {
            intakeB.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            intakeF.setPower(0);
        } else {
            intakeF.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            intakeB.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        }

        if(!gamepad1.left_bumper)
        {
            if (gamepad1.dpadUpWasPressed() && !gamepad1.left_bumper && (sorter.servoL.getPosition() < 0.5 && sorter.servoRF.getPosition() < 0.5 && sorter.servoRR.getPower() < 0.5)) {
                sorter.startTransfer(Sorter.detectedColor.GREEN);
            }
            if (gamepad1.dpadDownWasPressed() && !gamepad1.left_bumper && (sorter.servoL.getPosition() < 0.5 && sorter.servoRF.getPosition() < 0.5 && sorter.servoRR.getPower() < 0.5)) {
                sorter.startTransfer(Sorter.detectedColor.PURPLE);
            }


            sorter.updateTransfer();
        }

        if(gamepad1.left_bumper)
        {
            if (gamepad1.dpad_up && (sorter.servoL.getPosition() < 0.5 && sorter.servoRR.getPower() < 0.5))
            {sorter.servoRF.setPosition(0.53);}
            else{sorter.servoRF.setPosition(0.06);}
            if (gamepad1.dpad_down && (sorter.servoL.getPosition() < 0.5 && sorter.servoRF.getPosition() < 0.5))
            {sorter.servoRR.setPower(0.7);}
            else{sorter.servoRR.setPower(-0.016);}
            if (gamepad1.dpad_left && (sorter.servoRF.getPosition() < 0.5 && sorter.servoRR.getPower() < 0.5))
            {sorter.servoL.setPosition(0.8);}
            else{sorter.servoL.setPosition(0.385);}
        }



        if (sorter.slotL.getBallColor() != Sorter.detectedColor.NOTHING) { balls++; }
        if (sorter.slotRF.getBallColor() != Sorter.detectedColor.NOTHING) { balls++; }
        if (sorter.slotRR.getBallColor() != Sorter.detectedColor.NOTHING) { balls++; }

        if (balls == 0) { led.setPosition(0.5); }
        if (balls == 1) { led.setPosition(1); }
        if (balls == 2) { led.setPosition(0.6); }
        if (balls == 3) { led.setPosition(0.28); }

        balls = 0;

        if (cam.seenId == 21) {
            pattern = Pattern.GPP;
        }
        if (cam.seenId == 22) {
            pattern = Pattern.PGP;
        }
        if (cam.seenId == 23) {
            pattern = Pattern.PPG;
        }


        LLResult result = cam.camera.getLatestResult();
        if (result == null || !result.isValid()){llvresult = false;}
        else{llvresult = true;}

        if(llvresult)
        {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty())
            {
                double x1 = fiducials.get(0).getTargetCorners().get(0).get(0);
                double y1 = fiducials.get(0).getTargetCorners().get(0).get(1);
                double x2 = fiducials.get(0).getTargetCorners().get(1).get(0);
                double y2 = fiducials.get(0).getTargetCorners().get(1).get(1);
                double x3 = fiducials.get(0).getTargetCorners().get(2).get(0);
                double y3 = fiducials.get(0).getTargetCorners().get(2).get(1);
                double x4 = fiducials.get(0).getTargetCorners().get(3).get(0);
                double y4 = fiducials.get(0).getTargetCorners().get(3).get(1);
                double corner1 = Math.pow(Math.pow(x1-x4,2)+Math.pow(y1-y4,2),0.5);
                double corner2 = Math.pow(Math.pow(x2-x3,2)+Math.pow(y2-y3,2),0.5);
                telemetryP.addData("Left Corner = ", corner1);
                telemetryP.addData("Right Corner = ", corner2);
                telemetryP.addData("Average Corner = ", (corner1+corner2)/2);
                double x = (corner1+corner2)/2;
                double y = -0.00407296 * Math.pow(x,3) + 1.17902 * Math.pow(x,2) - 119.1841 * x + 7583.71974;
                //122 - 3200,0, 110 - 3300,20, 100 - 3400,30, 90 - 3450, 35, 80 - 3500,40, 70 - 3600,50, 60 - 3800,65, 50 - 4100,80, 40 - 4400,100, 35 - 4700,125
                double z = -0.000369038* Math.pow(x,3) + 0.0964091 * Math.pow(x,2) - 9.01097 * x + 335.01112;

                telemetryP.addData("Turret power = ", y);
                telemetryP.addData("Turret angle = ", z);
                if(gamepad1.rightBumperWasPressed())
                {
                    autoAdjacement = !autoAdjacement;
                }
                if(autoAdjacement)
                {
                    autoAim = true;
                    data = shooter.createCanonData(y, z);
                }
            }

        }

        if (gamepad1.squareWasPressed()) {
            autoAdjacement = false;
            data = shooter.createCanonData(3300, 0);
            servo.setPower(-0.7);
            autoAim = true;
        }
        if (gamepad1.triangleWasPressed()) {
            autoAdjacement = false;
            data = shooter.createCanonData(4100, 60);
            autoAim = true;
        }
        if (gamepad1.circleWasPressed()) {
            autoAdjacement = false;
            data = shooter.createCanonData(5000,140);
            autoAim = true;
        }
        if (gamepad1.crossWasPressed()) {
            autoAdjacement = false;
            data = shooter.createCanonData(0, 0);
            autoAim = false;
        }
        shooter.shootON(data);

        if(gamepad2.share){follower.setPose(new Pose (follower.getPose().getX(),follower.getPose().getY(), Math.toRadians(0)));telemetry.addData("Used", getRuntime());}


        telemetryP.addData("ColorL", sorter.slotL.getBallColor());
        telemetryP.addData("ColorRR", sorter.slotRR.getBallColor());
        telemetryP.addData("ColorRF", sorter.slotRF.getBallColor());
        telemetryP.addData("Pattern", pattern);
        telemetry.addData("ColorL", sorter.slotL.getBallColor());
        telemetry.addData("ColorRR", sorter.slotRR.getBallColor());
        telemetry.addData("ColorRF", sorter.slotRF.getBallColor());
        telemetry.addData("TransferActive", sorter.transferActive);
        telemetry.addData("DesiredColor", sorter.desiredColor);
        telemetry.addData("Shooter target RPM", shooter.getTargetRpm());
        telemetry.addData("Shooter meas RPM", shooter.getMeasuredRpm());
        telemetry.addData("Shooter power", shooter.getLastPower());
        telemetry.addData("VelocityX", follower.drivetrain.xVelocity());
        telemetry.addData("Loop", getRuntime());
        telemetry.addData("LSY", gamepad1.left_stick_y);
        telemetry.addData("share", gamepad1.share);
        telemetry.update();
        telemetryP.update();

    }

    private void shootOneBall(Sorter.detectedColor color) {
        if (!sorter.transferActive) {
            sorter.startTransfer(color);
            ballsShot++;  // Увеличиваем счетчик выстреленных мячей
            telemetry.addData("Balls Shot", ballsShot);  // Логирование количества выстреленных мячей
            telemetry.update();
        }
    }
}
