package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="LM1_October_26_DriverControl", group = "A_LM1")
public class LM_October_26_DriverControl extends OpMode {
    private DcMotor specimen_slides_motor;
    private DcMotor hang_motor;
    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;
    private CRServo intake_servo;
    private CRServo hang_servo;
    private Servo bucket_servo;
    private DcMotor intake_motor;
    private DcMotor outtake_slides_motor;

    private TouchSensor touchSensor;

    private double driveLeftX_debugger;
    private double driveLeftY_debugger;
    private double driveRightX_debugger;

    private double joyStickMargin = 0.004;

    private double intake_value = 1;
    private double outtake_value = -1;

    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;

    //intake arm positions
    private static double POSITION_TO_MOVE_UP = 10.0;
    private static double POSITION_TO_MOVE_DOWN = 1.0;

    //adjust servo positions for bucket
    private double pick_position = 0.25;
    private double drop_position = 0.995;
    private boolean bucket_was_pressed = true;

    private boolean bucket_one_click_debugger = true;

    //intake toggle variables
    private boolean intake_last_dropped = false;
    private boolean intake_was_pressed = false;

    //intake bucket variables
    private boolean sample_last_picked = false;
    private boolean trigger_was_pressed = false;

    //intake bucket variables
    private boolean allow_intake_motor_to_spin = true;
    private boolean sample_intaked = false;

    private boolean intake_hold = true;

    private int targetPosition;

    private String targetPositionNull_Debugger;

    private int slides_rest_position = 20;

    private boolean going_down = false;

    int slides_variation_margin = 30;

    private int variation_check_debugger = 0;

    private int last_slides_currentPosition = slides_rest_position;

    private ElapsedTime intake_timer = new ElapsedTime();
    private boolean score_once_debug_solution = true;

    private ElapsedTime slides_timer = new ElapsedTime();

    private ElapsedTime gametime = new ElapsedTime();

    private boolean prev_gamepad2a = false;
    private boolean cur_gamepad2a = false;

    private ElapsedTime hangtime = new ElapsedTime();

    private ElapsedTime intake_touchSensor_timer = new ElapsedTime();

    @Override
    public void init() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        specimen_slides_motor = hardwareMap.get(DcMotor.class, "specimen_slides_motor");
        specimen_slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang_motor = hardwareMap.get(DcMotor.class, "hang_motor");
        hang_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_servo = hardwareMap.get(CRServo.class, "intake_servo");
        intake_servo.setDirection(DcMotorSimple.Direction.REVERSE);

        bucket_servo = hardwareMap.get(Servo.class, "bucket_servo");
        bucket_servo.setDirection(Servo.Direction.REVERSE);
        hang_servo = hardwareMap.get(CRServo.class, "hang_servo");

        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtake_slides_motor = hardwareMap.get(DcMotor.class, "outtake_slides_motor");
        outtake_slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake_slides_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake_slides_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");
        // Set the mode of the touch sensor
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

    public void sampleintake() {

        if (gamepad2.left_trigger > 0.078) {
            intake_servo.setPower(0.995);
        }
        else if (gamepad2.right_trigger > 0.078 && allow_intake_motor_to_spin){
            intake_servo.setPower(-0.995);
            if(touchSensor.getValue() == 1 && allow_intake_motor_to_spin && intake_motor.getTargetPosition() != 0) {
                intake_touchSensor_timer.reset();
                allow_intake_motor_to_spin = false;
            }
        } else {
            intake_servo.setPower(0);
        }

        if (allow_intake_motor_to_spin == false && intake_touchSensor_timer.milliseconds() > 175) {
            allow_intake_motor_to_spin = true;
        }

        if (touchSensor.getValue() == 1) {
            telemetry.addData("Touch Sensor", "Pressed");
        } else {
            telemetry.addData("Touch Sensor", "Not Pressed");
        }

        if (gamepad2.left_bumper && !intake_was_pressed) {
            intake_last_dropped = !intake_last_dropped;

            if (intake_last_dropped) {
                //drop position near bucket
                score_once_debug_solution = false;
                allow_intake_motor_to_spin = true;
                intake_hold = true;
                intake_timer.reset();
            } else {
                //pick position extended
                intake_timer.reset();
                intake_hold = false;
                score_once_debug_solution = true;
                allow_intake_motor_to_spin = true;
                intake_motor.setTargetPosition(-225);
                intake_motor.setPower(0.35);
                intake_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        intake_was_pressed = gamepad2.left_bumper;

        if (intake_hold == true) {
            /*holds at drop position near bucket
            ~~~ 0 is start position as well and that gets in the way sometimes when a force (inertial or intentional/accidental external)
            is applied by falling back down and not going back up so an alternating value of 1 is also needed */

            if (intake_motor.getCurrentPosition() > -50 && intake_timer.milliseconds() > 1) {
                intake_motor.setTargetPosition(0);
                intake_motor.setPower(0);
                intake_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (intake_motor.getCurrentPosition() < -90 && intake_timer.milliseconds() > 1) {
                intake_motor.setTargetPosition(0);
                intake_motor.setPower(0.4); //abrupt amt of power, get it where it need to be, and then rest
                intake_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (intake_motor.getCurrentPosition() < -15 && intake_timer.milliseconds() > 1) {
                intake_motor.setTargetPosition(0);
                intake_motor.setPower(0);
                intake_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

        }
        else if(intake_hold && intake_motor.getTargetPosition() == -186 && intake_motor.getCurrentPosition() < -100) {
            intake_motor.setPower(0);
        }

        telemetry.addData("target pos", intake_motor.getTargetPosition());
        telemetry.addData("current pos", intake_motor.getCurrentPosition());
    }


    public void samplebucket() {

        if (going_down == false && outtake_slides_motor.getCurrentPosition() > -5) {
            if (last_slides_currentPosition <= (outtake_slides_motor.getCurrentPosition() + slides_variation_margin) && last_slides_currentPosition >= (outtake_slides_motor.getCurrentPosition() - slides_variation_margin)) {
                if (variation_check_debugger == 1) {
                    outtake_slides_motor.setPower(0);
                    variation_check_debugger = 0;
                }
                variation_check_debugger++;
            }
        }

        last_slides_currentPosition = outtake_slides_motor.getCurrentPosition();


        if (gametime.nanoseconds() > 5000 && ((gamepad2.right_bumper && !bucket_was_pressed && bucket_one_click_debugger == true) || bucket_one_click_debugger == false)) {
            if (bucket_one_click_debugger == true) {
                slides_timer.reset();
            }
            bucket_one_click_debugger = false;

            //outtake sample
            if (slides_timer.milliseconds() > 0 && slides_timer.milliseconds() < 1000) {
                bucket_servo.setPosition(drop_position);
            }

            //bucket is reset
            if (slides_timer.milliseconds() > 1000 && slides_timer.milliseconds() < 2000) {
                bucket_servo.setPosition(pick_position);
            }

            //slides down
            if (slides_timer.milliseconds() > 2000 && slides_timer.milliseconds() < 7000) {
                targetPosition = slides_rest_position;
                targetPositionNull_Debugger = String.valueOf(targetPosition); //prevents null value from being set in setTargetPosition
                if (targetPositionNull_Debugger != null) {
                    outtake_slides_motor.setTargetPosition(targetPosition);
                    outtake_slides_motor.setPower(0.995);
                    outtake_slides_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                going_down = false;
            }

            if (slides_timer.milliseconds() > 7000) { // allows it to be clicked AFTER everything is finished
                bucket_was_pressed = true;
                bucket_one_click_debugger = true;
            }
        }
        else if (gametime.nanoseconds() <= 5000) {
            bucket_was_pressed = gamepad2.right_bumper;
        }

        if (bucket_one_click_debugger == true) {
            bucket_was_pressed = gamepad2.right_bumper; //whether the preset can be triggered or not is conditional
        }

        //last_slides_currentPosition = outtake_slides_motor.getCurrentPosition();


        telemetry.addData("bucket debugger: ", bucket_one_click_debugger);
        telemetry.addData("right_bumper: ", gamepad2.right_bumper);
        telemetry.addData("!bucked_was_pressed", !bucket_was_pressed);
        telemetry.addData("slides_pos: ", outtake_slides_motor.getCurrentPosition());
    }


    public void sampleslides() {

        if (gamepad2.right_stick_y < -0.25) {
            targetPosition = -4874;
            targetPositionNull_Debugger = String.valueOf(targetPosition); //prevents null value from being set in setTargetPosition
            if (targetPositionNull_Debugger != null) {
                outtake_slides_motor.setTargetPosition(targetPosition);
                outtake_slides_motor.setPower(0.995);
                outtake_slides_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            going_down = true;
        } else if (gamepad2.right_stick_y > 0.25) {
            targetPosition = slides_rest_position;
            targetPositionNull_Debugger = String.valueOf(targetPosition); //prevents null value from being set in setTargetPosition
            if (targetPositionNull_Debugger != null) {
                outtake_slides_motor.setTargetPosition(targetPosition);
                outtake_slides_motor.setPower(0.995);
                outtake_slides_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            going_down = false;
        }
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

    public void climbactuator() { //drive motors
        if (gamepad2.dpad_up) {
            //move actuator up
            hang_motor.setPower(-0.995); //switch if needed
        } else if (gamepad2.dpad_down) {
            //move actuator down
            hang_motor.setPower(0.995);
        } else {
            //if nothing slides-related is happening, stop all movement
            hang_motor.setPower(0);
        }

        prev_gamepad2a = cur_gamepad2a;
        cur_gamepad2a = gamepad2.a;
        if (cur_gamepad2a && !prev_gamepad2a) {
            hang_servo.setPower(0.5);
            if (hang_servo.getPower() > 0.1 && hang_servo.getPower() < 1) {
                hang_servo.setPower(1);
                hangtime.reset();
            }
            if (hangtime.milliseconds() >= 1000) {
                hang_motor.setPower(0);
            }
        }
    }


    @Override
    public void loop() {

        samplebucket(); //should be called BEFORE sampleslides()
        sampleslides(); //should be called AFTER samplebucket()
        sampleintake();
        specimenslides();
        climbactuator();
        drivetrain();

    }
}