package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Bot2_Driver extends OpMode {

    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;
    private DcMotor specimen_slides_motor;

    private CRServo left_arm_servo;
    private CRServo right_arm_servo;

    private boolean cur_bucket = false;
    private boolean prev_bucket = false;
    private boolean toggle_bucket = false;

    private Servo left_bucket;
    private Servo right_bucket;

    private CRServo intake;
    private Servo left_intake_servo;
    private Servo right_intake_servo;

    private DcMotor climb;

    private double driveLeftX_debugger;
    private double driveLeftY_debugger;
    private double driveRightX_debugger;

    private double joyStickMargin = 0.004;

    @Override
    public void init() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        specimen_slides_motor = hardwareMap.get(DcMotor.class, "specimen_slides_motor");
        specimen_slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_arm_servo = hardwareMap.get(CRServo.class, "left_arm_servo");
        right_arm_servo = hardwareMap.get(CRServo.class, "right_arm_servo");
        left_bucket = hardwareMap.get(Servo.class, "left_bucket");
        right_bucket = hardwareMap.get(Servo.class, "right_bucket");
        intake = hardwareMap.get(CRServo.class, "intake");
        climb = hardwareMap.get(DcMotor.class, "climb");
        right_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_bucket.setPosition(0);
        right_bucket.setPosition(0);

    }

    public void drivetrain() {

        if (Math.abs(gamepad1.left_stick_x) >= joyStickMargin) {
            driveLeftX_debugger = gamepad1.left_stick_x;
        }
        else {
            driveLeftX_debugger = 0;
        }

        if (Math.abs(gamepad1.left_stick_y) >= joyStickMargin) {
            driveLeftY_debugger = gamepad1.left_stick_y;
        }
        else {
            driveLeftY_debugger = 0;
        }

        if (Math.abs(gamepad1.right_stick_x) >= joyStickMargin) {
            driveRightX_debugger = gamepad1.right_stick_x;
        }
        else {
            driveRightX_debugger = 0;
        }

        left_front.setPower(driveLeftY_debugger - driveRightX_debugger - driveLeftX_debugger);
        left_back.setPower(driveLeftY_debugger - driveRightX_debugger + driveLeftX_debugger);
        right_front.setPower(driveLeftY_debugger + driveRightX_debugger + driveLeftX_debugger);
        right_back.setPower(driveLeftY_debugger + driveRightX_debugger - driveLeftX_debugger);
    }

    public void bucket(){
        cur_bucket = gamepad2.a;

        if (cur_bucket && !prev_bucket) {
            toggle_bucket = !toggle_bucket;

            if (toggle_bucket){
                left_bucket.setPosition(0.7);
                right_bucket.setPosition(0.7);
            } else {
                left_bucket.setPosition(0);
                right_bucket.setPosition(0);
            }
        }
        prev_bucket = cur_bucket;
    }

    public void specimenslides() {
        if (gamepad2.left_stick_y > 0.1) {
            //move slides up
            specimen_slides_motor.setPower(-0.6);
        } else if (gamepad2.left_stick_y < -0.1) {
            //move slides down
            specimen_slides_motor.setPower(0.6);
        } else {
            specimen_slides_motor.setPower(0);
        }
    }

    public void extend_arm(){
        if (gamepad2.b){
            left_arm_servo.setPower(0.995);
            right_arm_servo.setPower(0.995);
        } else if (gamepad2.x){
            left_arm_servo.setPower(-0.995);
            right_arm_servo.setPower(-0.995);
        } else{
            left_arm_servo.setPower(0);
            right_arm_servo.setPower(0);
        }

    }

    public void climb(){
        if (gamepad2.right_stick_y> 0.1){
            climb.setPower(0.6);
        } else if (gamepad2.right_stick_y < -0.1){
            climb.setPower(-0.6);
        } else{
            climb.setPower(0);
        }
    }

    public void intake(){
        if (gamepad2.left_bumper){
            intake.setPower(0.995);
        } else if (gamepad2.right_bumper){
            intake.setPower(-0.995);
        } else{
            intake.setPower(0);
        }

        if (gamepad2.dpad_up){

        }
    }

    @Override
    public void loop() {
        bucket();
        specimenslides();
        extend_arm();
        climb();
        intake();
        drivetrain();
    }


}
