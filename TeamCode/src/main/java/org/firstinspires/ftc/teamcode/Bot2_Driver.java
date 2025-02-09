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

@TeleOp(name = "Bot2_Driver")
public class Bot2_Driver extends OpMode {

    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;
    private DcMotor specimen_slides_motor;


    private int SPECIMENUPOSITION = -630;
    private int SPECIMENDOWNPOSITION = 50;

    private DcMotor left_arm_motor;
    private DcMotor right_arm_motor;

    private int SLIDESUPOSITION = 1000000;
    private int SLIDESDOWNPOSITION = -30;


    private boolean cur_bucket = false;
    private boolean prev_bucket = false;
    private boolean toggle_bucket = false;

    private Servo left_bucket;
    private Servo right_bucket;

    private DcMotor intake;
    private Servo left_intake_chamber_servo;
    private Servo right_intake_chamber_servo;
    private Servo left_extendo_servo;
    private Servo right_extendo_servo;

    private boolean cur_extendo = false;
    private boolean prev_extendo = false;
    private boolean toggle_extendo = false;

//    private CRServo left_climb;
//    private CRServo right_climb;

    private double driveLeftX_debugger;
    private double driveLeftY_debugger;
    private double driveRightX_debugger;

    private double joyStickMargin = 0.004;

    private double ONETURNCONSTANT = 0.2155;
    ElapsedTime Bucketruntime = new ElapsedTime();
    ElapsedTime Chambertime = new ElapsedTime();

    @Override
    public void init() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        specimen_slides_motor = hardwareMap.get(DcMotor.class, "specimen_slides_motor");
        specimen_slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        specimen_slides_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_arm_motor = hardwareMap.get(DcMotor.class, "left_arm_motor");
        right_arm_motor = hardwareMap.get(DcMotor.class, "right_arm_motor");
        left_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_arm_motor.setDirection(DcMotor.Direction.REVERSE);
        left_bucket = hardwareMap.get(Servo.class, "left_bucket");
        right_bucket = hardwareMap.get(Servo.class, "right_bucket");
        right_bucket.setDirection(REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        left_intake_chamber_servo = hardwareMap.get(Servo.class, "left_intake_servo");
        right_intake_chamber_servo = hardwareMap.get(Servo.class, "right_intake_servo");
     //   left_intake_chamber_servo.setDirection(REVERSE);
        left_extendo_servo = hardwareMap.get(Servo.class, "left_extendo_servo");
        right_extendo_servo = hardwareMap.get(Servo.class, "right_extendo_servo");
        right_extendo_servo.setDirection(REVERSE);
//        left_extendo_servo.setPosition(0);
//        right_extendo_servo.setPosition(0);
//        left_climb = hardwareMap.get(CRServo.class, "left_climb");
//        right_climb = hardwareMap.get(CRServo.class, "right_climb");

        right_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bucketruntime.reset();
        Chambertime.reset();


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
        prev_bucket = cur_bucket;
        cur_bucket = gamepad2.a;

        if (cur_bucket && !prev_bucket) {

            if (toggle_bucket){
                right_bucket.setDirection(REVERSE);
                left_bucket.setDirection(FORWARD);
                //left_bucket.setPosition(0.080625);
                right_bucket.setPosition(0.080625);

            } else {
                right_bucket.setDirection(FORWARD);
                left_bucket.setDirection(REVERSE);
                //left_bucket.setPosition(0);
                right_bucket.setPosition(0);
           }
            toggle_bucket = !toggle_bucket;
        }

    }

    public void specimenslides() {
        if (gamepad2.left_stick_y < -0.8) {
            specimen_slides_motor.setTargetPosition(SPECIMENUPOSITION);
            specimen_slides_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            specimen_slides_motor.setPower(0.7);
        } else if (gamepad2.left_stick_y > 0.8) {
            specimen_slides_motor.setTargetPosition(SPECIMENDOWNPOSITION);
            specimen_slides_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            specimen_slides_motor.setPower(0.7);
        }
        if (specimen_slides_motor.getCurrentPosition() > -50 && specimen_slides_motor.getTargetPosition() == 0){
            specimen_slides_motor.setPower(0);
        }
//         else {
//            specimen_slides_motor.setPower(0);
//        }
    }

    public void extend_slides(){
        if (gamepad2.right_stick_y < -0.8) {
            left_arm_motor.setTargetPosition(SLIDESUPOSITION);
            right_arm_motor.setTargetPosition(SLIDESUPOSITION);
        } else if (gamepad2.right_stick_y > 0.8) {
            left_arm_motor.setTargetPosition(SLIDESDOWNPOSITION);
            right_arm_motor.setTargetPosition(SLIDESDOWNPOSITION);
        } else {
            return;
        }
        left_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start moving both motors
        left_arm_motor.setPower(1);
        right_arm_motor.setPower(1);


        if (left_arm_motor.getCurrentPosition() > -50 || right_arm_motor.getCurrentPosition() > -50 && left_arm_motor.getTargetPosition() == 0 || right_arm_motor.getTargetPosition() == 0){
            left_arm_motor.setPower(0);
            right_arm_motor.setPower(0);
        }

    }

//    public void climb(){
//        if (gamepad2.right_stick_y > 0.1){
//            left_climb.setPower(gamepad2.right_stick_y);
//            right_climb.setPower(-gamepad2.right_stick_y);
//        } else{
//            left_climb.setPower(0);
//            right_climb.setPower(0);
//        }
//    }

    public void intake(){
        if (gamepad2.left_bumper){
            intake.setPower(0.5);
        } else if (gamepad2.right_bumper){
            intake.setPower(-0.995);
        } else{
            intake.setPower(0);
        }

        prev_extendo = cur_extendo;
        cur_extendo = gamepad2.dpad_up;

        if (cur_extendo && !prev_extendo){

            toggle_extendo = !toggle_extendo;

            if (toggle_extendo == true){
                Chambertime.reset();
                left_intake_chamber_servo.setPosition(1);
                //right_intake_chamber_servo.setPosition(0.2155);
                if (Chambertime.milliseconds() > 5000){
                    left_extendo_servo.setPosition(0.05673);
                    //right_extendo_servo.setPosition(0.056208);
                }
            }
            if (toggle_extendo == false) {
                left_extendo_servo.setPosition(0);
                //right_extendo_servo.setPosition(0);
                left_intake_chamber_servo.setPosition(0);
                //right_intake_chamber_servo.setPosition(0);
            }


        }

    }




    @Override
    public void loop() {
        bucket();
        specimenslides( );
        extend_slides();
       //
        // climb();
        intake();
        drivetrain();
        telemetry.addData("right_extendo_servo", right_extendo_servo.getPosition());
        telemetry.addData("left_extendo_servo", left_extendo_servo.getPosition());
        telemetry.addData("toggle_extendo_state", toggle_extendo);
        telemetry.addData("slides_motor_position", specimen_slides_motor.getCurrentPosition());
        telemetry.addData("slides_motor_target_position", specimen_slides_motor.getTargetPosition());
        telemetry.addData("slides_motor_power", specimen_slides_motor.getPower());
        telemetry.addData("toggle_bucket", toggle_bucket);
//        left_arm_motor.setPower(0.7);
//        right_arm_motor.setPower(0.7);
//        telemetry.addData("left_bucket", left_arm_motor.getCurrentPosition());
//        telemetry.addData("right_bucket", right_arm_motor.    getCurrentPosition());
//        telemetry.update();
    }


}