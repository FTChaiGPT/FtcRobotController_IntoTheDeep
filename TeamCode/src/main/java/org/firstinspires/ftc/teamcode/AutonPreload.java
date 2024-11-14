package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "AutonPreload", group = "LM1_October_26")

public class AutonPreload extends LinearOpMode {

    private DcMotorEx left_front, left_back, right_front, right_back;

    private DcMotor outtake_slides_motor;
    private CRServo intake_servo;
    private Servo bucket_servo;
    private DcMotor intake_motor;
    private DcMotor hang_motor;




    @Override
    public void runOpMode() {

        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        right_back = hardwareMap.get(DcMotorEx.class, "right_back");

        outtake_slides_motor = hardwareMap.get(DcMotor.class, "outtake_slides_motor");
        outtake_slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_servo = hardwareMap.get(CRServo.class, "intake_servo");
        intake_servo.setDirection(DcMotorSimple.Direction.REVERSE);

        bucket_servo = hardwareMap.get(Servo.class, "bucket_servo");
        bucket_servo.setDirection(Servo.Direction.REVERSE);

        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake_motor.setTargetPosition(0);

        outtake_slides_motor = hardwareMap.get(DcMotor.class, "outtake_slides_motor");
        outtake_slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake_slides_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake_slides_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hang_motor = hardwareMap.get(DcMotor.class, "hang_motor");
        hang_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d Pose2 = new Pose2d(24, 0, Math.toRadians(0));
        Pose2d Pose3 = new Pose2d(-60, -52, Math.toRadians(0));
        Pose2d Pose4 = new Pose2d(-60, -36, Math.toRadians(-45));
        Pose2d resetBotPos = new Pose2d(0, 0, Math.toRadians(0));




        Trajectory Position1 = drive.trajectoryBuilder(startPose)
                .forward(3)
                //.splineTo(new Vector2d(9, 20), Math.toRadians(0))
                .build();
        Trajectory Position2 = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(7, 24), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(5, 24), Math.toRadians(0))
                //.splineTo(new Vector2d(9, 20), Math.toRadians(0))
                .build();
        Trajectory Position3 = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(-6,0), Math.toRadians(0))
                //.splineTo(new Vector2d(9, 20), Math.toRadians(0))
                .build();
        Trajectory PickSample11 = drive.trajectoryBuilder(resetBotPos)
                .forward(16.75)


                //.splineToConstantHeading(new Vector2d(-20, 0), Math.toRadians(0))
                //.splineToLinearHeading(new Vector2d(-60, -36, Math.toRadians(-45)), Math.toRadians(0))
                .build();
        Trajectory PickSample13 = drive.trajectoryBuilder(PickSample11.end())
                .forward(7)
                //.splineToConstantHeading(new Vector2d(-20, 0), Math.toRadians(0))
                //.splineToLinearHeading(new Vector2d(-60, -36, Math.toRadians(-45)), Math.toRadians(0))
                .build();
        Trajectory PickSample12 = drive.trajectoryBuilder(resetBotPos)
                .back(2)
                //.splineToConstantHeading(new Vector2d(-20, 0), Math.toRadians(0))
                //.splineToLinearHeading(new Vector2d(-60, -36, Math.toRadians(-45)), Math.toRadians(0))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(startPose);
        drive.followTrajectory(Position1);
        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(Position2);
        drive.turn(Math.toRadians(-45.5));
        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(Position3);
        outtake_slides_motor.setPower(-0.995);
        sleep(1850);
        bucket_servo.setPosition(0.995);
        sleep(1000);
        drive.setPoseEstimate(resetBotPos);
        outtake_slides_motor.setPower(0.995);
        bucket_servo.setPosition(0.25);
        sleep(2000);

        intake_motor.setPower(-0.15);
        sleep(1500);
        intake_motor.setPower(0);
        drive.followTrajectory(PickSample11);
        drive.turn(Math.toRadians(52));
        //drive.followTrajectory(PickSample13);
        intake_servo.setPower(-0.995);
        sleep(1700);
        intake_motor.setPower(0.15);
        sleep(1500);
        intake_motor.setPower(0);
        intake_servo.setPower(0.995);
        sleep(1700);
        intake_servo.setPower(0);

        drive.turn(Math.toRadians(-46));
        drive.followTrajectory(PickSample12);
        outtake_slides_motor.setPower(-0.995);
        sleep(1850);
        bucket_servo.setPosition(0.995);
        sleep(1000);
        drive.setPoseEstimate(resetBotPos);
        outtake_slides_motor.setPower(0.995);
        bucket_servo.setPosition(0.25);
        sleep(1900);

        drive.followTrajectory(PickSample13);
        intake_motor.setPower(-0.15);
        sleep(2000);
        intake_motor.setPower(0);
        drive.turn(Math.toRadians(85));
        intake_servo.setPower(-0.995);
        sleep(1700);
        intake_motor.setPower(0.15);
        sleep(2000);
        intake_motor.setPower(0);
        intake_servo.setPower(0.995);
        sleep(1200);
        intake_servo.setPower(0);

        drive.turn(Math.toRadians(-85));
        drive.followTrajectory(PickSample12);

        outtake_slides_motor.setPower(-0.995);
        sleep(1850);
        bucket_servo.setPosition(0.995);
        sleep(1000);
        drive.setPoseEstimate(resetBotPos);
        outtake_slides_motor.setPower(0.995);
        bucket_servo.setPosition(0.25);
        sleep(2000);

        hang_motor.setPower(-0.995);
        sleep(3000);
        hang_motor.setPower(0);

//        outtake_slides_motor.setPower(-0.995);
//        sleep(2200);
//        bucket_servo.setPosition(0.995);
//        sleep(1500);
//        drive.setPoseEstimate(resetBotPos);
//        outtake_slides_motor.setPower(0.995);
//        bucket_servo.setPosition(0.25);
//        sleep(1900);






//        intake_motor.setPower(-0.2);
//        sleep(1200);
//        intake_motor.setPower(0);
//        drive.followTrajectory(PickSample12);
//        drive.turn(Math.toRadians(48));
//
//        intake_servo.setPower(-0.995);
//        sleep(2000);
//        intake_motor.setPower(0.2);
//        sleep(1400);
//        intake_motor.setPower(0);
//        intake_servo.setPower(0.995);
//        sleep(1200);
//        intake_servo.setPower(0);






//        bucket_servo.setPosition(0.25);
//        outtake_slides_motor.setPower(-0.995);
//        drive.followTrajectory(PickSample1);
//        drive.setPoseEstimate(Pose3);
//
//
//        intake_motor.setPower(-0.2);
//        sleep(1200);
//        intake_motor.setPower(0);
//        intake_servo.setPower(-0.995);
//        sleep(1900);
//        intake_servo.setPower(0);
//        intake_motor.setPower(0.2);
//        sleep(1100);
//        intake_motor.setPower(0);
//        intake_servo.setPower(-0.995);
//        sleep(1200);
//        intake_servo.setPower(0);
//
//        drive.followTrajectory(SplinetoPosition);
//        outtake_slides_motor.setPower(0.995);
//        sleep(3000);
//        bucket_servo.setPosition(0.995);
//        sleep(1000);
//        bucket_servo.setPosition(0.25);
//        outtake_slides_motor.setPower(-0.995);
//        drive.followTrajectory(PickSample1);
//        drive.setPoseEstimate(Pose3);
//
//




    }
}