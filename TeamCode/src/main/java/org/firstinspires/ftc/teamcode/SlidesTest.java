package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "SlidesTest")
public class SlidesTest extends OpMode {

    private DcMotor left_arm_motor;
    private DcMotor right_arm_motor;

    private final static int SLIDES_UPOSITION = 5000;
    private final static int SLIDES_DOWN_POSITION = 20;
    private final static int SLIDES_MINIMUM_THRESHOLD = 55;

    @Override
    public void init() {

        left_arm_motor = hardwareMap.get(DcMotor.class, "left_arm_motor");
        right_arm_motor = hardwareMap.get(DcMotor.class, "right_arm_motor");
        left_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_arm_motor.setDirection(DcMotor.Direction.REVERSE);
        left_arm_motor.setTargetPosition(SLIDES_DOWN_POSITION);
        right_arm_motor.setTargetPosition(SLIDES_DOWN_POSITION);
    }


    public void extend_slides(){
        if (gamepad2.right_stick_y <= -0.5) {
            left_arm_motor.setTargetPosition(SLIDES_UPOSITION);
            right_arm_motor.setTargetPosition(SLIDES_UPOSITION);
        } else if (gamepad2.right_stick_y >= 0.5) {
            left_arm_motor.setTargetPosition(SLIDES_DOWN_POSITION);
            right_arm_motor.setTargetPosition(SLIDES_DOWN_POSITION);
        }
        if (Math.abs(gamepad2.right_stick_y) >= 0.5) {
            left_arm_motor.setPower(1);
            right_arm_motor.setPower(1);
        }
        left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (left_arm_motor.getCurrentPosition() <= SLIDES_MINIMUM_THRESHOLD && left_arm_motor.getTargetPosition() == SLIDES_DOWN_POSITION){
            left_arm_motor.setPower(0);
            right_arm_motor.setPower(0);
        }
    }


    @Override
    public void loop() {
        extend_slides();
        telemetry.addData("left current", left_arm_motor.getCurrentPosition());
        telemetry.addData("left target", left_arm_motor.getTargetPosition());
        telemetry.addLine(" ");
        telemetry.addData("right current", right_arm_motor.getCurrentPosition());
        telemetry.addData("right target", right_arm_motor.getTargetPosition());
    }
}