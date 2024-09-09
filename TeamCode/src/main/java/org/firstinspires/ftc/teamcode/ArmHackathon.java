package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Arm")
public class ArmHackathon extends OpMode {
    private DcMotor slides_motor;
    private DcMotor slidesPivot_motor;
    private double ticks = 537.7;
    private double newTarget;
    private double armHeight = 0;
    private double armAngle = 0;
    private double slidesSpeed = 0.5;


    @Override
    public void init() {
        slides_motor = hardwareMap.get(DcMotor.class, "slides_motor");
        slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesPivot_motor = hardwareMap.get(DcMotor.class, "slides_pivot");
        slidesPivot_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesPivot_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void slidesMoveToPosition(double degrees) {
        slides_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticks * degrees;
        slides_motor.setTargetPosition((int)newTarget);
        slides_motor.setPower(0.5);
        slides_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void slidesPivotMoveToPosition(double degrees) {
        slidesPivot_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticks * degrees;
        slidesPivot_motor.setTargetPosition((int)newTarget);
        slidesPivot_motor.setPower(0.5);
        slidesPivot_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    @Override
    public void loop() {
        if (gamepad2.right_stick_y > 0.1) { //slides extend
            armHeight = armHeight + slidesSpeed;
            slides_motor.setPower(0.5);
        }
        else {
            slides_motor.setPower(0);
            slidesMoveToPosition(armHeight);
        }

        if (gamepad2.right_stick_y < -0.1) { //slides retract
            armHeight = armHeight - slidesSpeed;
            slides_motor.setPower(-0.5);
        }
        else {
            slides_motor.setPower(0);
            slidesMoveToPosition(armHeight);
        }

        if (gamepad2.left_stick_y > 0.1) { //slides pivot
            armAngle = armAngle + slidesSpeed;
            slidesPivot_motor.setPower(0.5);
        }
        else {
            slidesPivot_motor.setPower(0);
            slidesPivotMoveToPosition(armAngle);
        }

        if (gamepad2.left_stick_y < -0.1) { //slides pivot
            armAngle = armAngle - slidesSpeed;
            slidesPivot_motor.setPower(-0.5);

        }
        else {
            slidesPivot_motor.setPower(0);
            slidesPivotMoveToPosition(armAngle);
        }

        telemetry.addData("slides", slides_motor.getCurrentPosition());
        telemetry.addData("slides pivot", slidesPivot_motor.getCurrentPosition());
        telemetry.update();

    }
}