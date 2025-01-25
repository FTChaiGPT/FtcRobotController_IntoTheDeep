package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "ZbinZerboVone")
public class ZpinZervoOne extends OpMode {

    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;
    private DcMotor specimen_slides_motor;

    private DcMotor left_arm_motor;
    private DcMotor right_arm_motor;

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

    public static double ONE_TURN_CONSTANT = 0.2155;
    public static double TURN_AMOUNT = 90;
    public static double TIME_CONSTANT = 5500;


    @Override
    public void init() {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        specimen_slides_motor = hardwareMap.get(DcMotor.class, "specimen_slides_motor");
        specimen_slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_arm_motor = hardwareMap.get(DcMotor.class, "left_arm_motor");
        right_arm_motor = hardwareMap.get(DcMotor.class, "right_arm_motor");
        left_bucket = hardwareMap.get(Servo.class, "left_bucket");
        right_bucket = hardwareMap.get(Servo.class, "right_bucket");
        intake = hardwareMap.get(DcMotor.class, "intake");
        left_intake_chamber_servo = hardwareMap.get(Servo.class, "left_intake_servo");
        right_intake_chamber_servo = hardwareMap.get(Servo.class, "right_intake_servo");
        left_extendo_servo = hardwareMap.get(Servo.class, "left_extendo_servo");
        right_extendo_servo = hardwareMap.get(Servo.class, "right_extendo_servo");
//        left_climb = hardwareMap.get(CRServo.class, "left_climb");
//        right_climb = hardwareMap.get(CRServo.class, "right_climb");
        right_back.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @NonNull
    @Override
    public void loop() {

        right_extendo_servo.setPosition(0);

        sleep((long)TIME_CONSTANT);

        telemetry.addData("servo inputed value: ", translateToFiveTurnServoPosition(TURN_AMOUNT));
        left_extendo_servo.setPosition(translateToFiveTurnServoPosition(TURN_AMOUNT));
        telemetry.update();

        sleep((long)TIME_CONSTANT);
    }

    @NonNull
    public double translateToFiveTurnServoPosition(double degrees) {
        return (degrees / 360) * ONE_TURN_CONSTANT;
    }

}

