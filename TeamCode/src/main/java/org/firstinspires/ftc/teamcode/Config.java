package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.text.Format;
@Configurable
@TeleOp(name="config", group="Linear OpMode")

public class Config extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftRear = null, rightRear = null, leftfront = null, rightfront = null, intakeMotor = null, shooter1 = null, shooter2 = null, take = null;

    TelemetryManager telemetryM;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();




        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeF");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        take = hardwareMap.get(DcMotorEx.class, "intakeB");
        take.setDirection(DcMotor.Direction.REVERSE);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter_r");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter_l");

        shooter1.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.FORWARD);

        leftfront  = hardwareMap.get(DcMotorEx.class, "lf");
        rightfront = hardwareMap.get(DcMotorEx.class, "rf");
        leftRear  = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);


        int canon = 0;

        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        take.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        runtime.reset();
        int rr=0,rf=0,lf=0,lr=0,shooter_r=0,shooter_l=0,intake=0,outtake = 0;


        while (opModeIsActive()) {
            shooter1.setMotorEnable();
            shooter2.setMotorEnable();

            if (gamepad1.y){
                leftfront.setPower(0.5);
                lf =1;
            }
            else{ lf = 0;
                leftfront.setPower(0);

            }

            if (gamepad1.x){
                rightfront.setPower(0.5);
                rf = 1;
            }
            else{ rf = 0;rightfront.setPower(0);}

            if (gamepad1.a){
                rightRear.setPower(0.5);
                rr = 1;
            }
            else{ rr = 0;rightRear.setPower(0);}

            if (gamepad1.b){
                leftRear.setPower(0.5);
                lr = 1;
            }
            else{ lr = 0;leftRear.setPower(0);}
            if (gamepad1.left_bumper){
                shooter1.setPower(1);
                shooter_l = 1;
            }
            else{ shooter_l = 0;shooter1.setPower(0);}

            if (gamepad1.right_bumper){
                shooter2.setPower(1);
                shooter_r = 1;
            }
            else{ shooter_r = 0;shooter2.setPower(0);}

            if (gamepad1.right_trigger >0.1){
                intakeMotor.setPower(1);
                intake = 1;
            }
            else{ intake = 0;intakeMotor.setPower(0);}

            if (gamepad1.left_trigger >0.1){
                take.setPower(1);
                outtake = 1;
            }else{ outtake = 0;take.setPower(0);}
            telemetry.addData("lr",leftRear.getPower());
            telemetry.addData("lf",leftfront.getPower());
            telemetry.addData("shooter_r",shooter_r);
            telemetry.addData("shooter_l",shooter_l);
            telemetry.addData("intakeF",intake);
            telemetry.addData("intakeB",outtake);
            telemetry.addData("rr",rightRear.getPower());
            telemetry.addData("rf",rightfront.getPower());
            telemetry.update();

        }
    }
}
