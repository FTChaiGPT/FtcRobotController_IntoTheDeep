package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "FinalHackathonDrive")
public class FinalHackathonDrive extends OpMode {
    private Servo claw_servo;
    private CRServo clawPivot_crservo;
    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;
    private DcMotor hang_motor;
    private Servo hang_servo;
    private DcMotor slides_motor;
    private DcMotor slidespivot_motor;
    private boolean clawToggle = true;
    private boolean prev_gamepad2a = false;
    private boolean cur_gamepad2a = false;
    private boolean activated = false;

    @Override
    public void init() {
        left_front = hardwareMap.get(DcMotor.class, "front_left");
        left_back = hardwareMap.get(DcMotor.class, "back_left");
        right_front = hardwareMap.get(DcMotor.class, "front_right");
        right_back = hardwareMap.get(DcMotor.class, "back_right");
        claw_servo = hardwareMap.servo.get("claw_servo");
        clawPivot_crservo = hardwareMap.crservo.get("claw_pivot");
//        hang_motor = hardwareMap.get(DcMotor.class, "hang_motor");
////        hang_servo = hardwareMap.get(Servo.class, "hang_servo");
//        hang_motor.setDirection(DcMotor.Direction.FORWARD);
//        hang_servo.setDirection(Servo.Direction.FORWARD);
//        hang_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hang_servo.setPosition(0);
//        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.REVERSE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides_motor = hardwareMap.get(DcMotor.class, "slides_motor");
        slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides_motor.setPower(0);
        slidespivot_motor = hardwareMap.get(DcMotor.class, "slides_pivot");
        slidespivot_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

//    public void releaseHang() {
//        if (gamepad2.left_bumper) {
//            hang_servo.setPosition(0.995);
//        }
//    }
//    public void hangBot() {
//        if (gamepad2.right_bumper) {
//            hang_motor.setPower(1);
//            activated = true;
//        }
//        else if (activated == true) {
//            hang_motor.setPower(0.03);
//        }
//        else {
//            hang_motor.setPower(0);
//        }
//    }

    public void slides() { //drive motors
        if (gamepad2.right_stick_y > 0.1) {
            //move actuator up
            slides_motor.setPower(0.5);
            slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else if (gamepad2.right_stick_y < -0.1) {
            //move actuator down
            slides_motor.setPower(-0.5);
            slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else {
            //if nothing slides-related is happening, stop all movement
            slides_motor.setPower(0);
            slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void slidespivot() {
        if (gamepad2.left_stick_y > 0.1) {
            //move actuator up
            slidespivot_motor.setPower(0.5);
            slidespivot_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else if (gamepad2.left_stick_y < -0.1) {
            //move actuator down
            slidespivot_motor.setPower(-0.5);
            slidespivot_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else {
            //if nothing slides-related is happening, stop all movement
            slidespivot_motor.setPower(0);
            slidespivot_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
    }

    @Override
    public void loop() {
//        releaseHang();
//        hangBot();
        slidespivot();
        slides();

        if (gamepad2.b) {
            clawPivot_crservo.setPower(0.995);
        }
        else {
            clawPivot_crservo.setPower(0);
        }
        if (gamepad2.x){
            clawPivot_crservo.setPower(-0.995);
        }
        else{
            clawPivot_crservo.setPower(0);
        }

        prev_gamepad2a = cur_gamepad2a;
        cur_gamepad2a = gamepad2.a;

        if (cur_gamepad2a && !prev_gamepad2a) {
            if (clawToggle) { //clawToggle is equal true
                clawToggle = false;
                claw_servo.setPosition(0);
            }
            else {
                clawToggle = true;
                claw_servo.setPosition(1);
            }
        }

        left_front.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x);
        left_back.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x);
        right_front.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x);
        right_back.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x);
    }
}
