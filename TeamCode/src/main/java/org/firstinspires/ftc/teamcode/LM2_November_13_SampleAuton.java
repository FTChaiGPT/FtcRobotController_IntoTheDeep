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
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.DriveAdditionalActions.*;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.RoadrunnerAdditionalActions.TurnFast;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



@Autonomous (name = "LM2_November_13_SampleAuton", group = "A-LM2")

public class LM2_November_13_SampleAuton extends LinearOpMode {

    private DcMotorEx left_front;
    private DcMotorEx left_back;
    private DcMotorEx right_front;
    private DcMotorEx right_back;

    private DcMotor outtake_slides_motor;
    private CRServo intake_servo;
    private Servo bucket_servo;
    private DcMotor intake_motor;
    private DcMotor hang_motor;

    private VoltageSensor batteryVoltageSensor;

//    TurnFast robot = new TurnFast();
//    private VoltageSensor batteryVoltageSensor;
//    Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() {
        // batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        outtake_slides_motor = hardwareMap.get(DcMotor.class, "outtake_slides_motor");
        outtake_slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_servo = hardwareMap.get(CRServo.class, "intake_servo");

        bucket_servo = hardwareMap.get(Servo.class, "bucket_servo");
        bucket_servo.setDirection(Servo.Direction.REVERSE);

        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setDirection(DcMotor.Direction.REVERSE);
        intake_motor.setTargetPosition(0);

        outtake_slides_motor = hardwareMap.get(DcMotor.class, "outtake_slides_motor");
        outtake_slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake_slides_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake_slides_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hang_motor = hardwareMap.get(DcMotor.class, "hang_motor");
        hang_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Pose2d resetBotPose = new Pose2d(0, 0, 0);

        Trajectory moveToBucket = drive.trajectoryBuilder(resetBotPose)
                .splineToConstantHeading(new Vector2d(3, 0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(10.85, 20), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(5, 20), Math.toRadians(0))
                .build();

        Trajectory goneToBucket = drive.trajectoryBuilder(resetBotPose)
                .splineToConstantHeading(new Vector2d(-5, 0), Math.toRadians(0))
                .build();

        Trajectory goToBucket = drive.trajectoryBuilder(resetBotPose)
                .splineToConstantHeading(new Vector2d(-5, -3), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-6.5, -3), Math.toRadians(0))
                .build();

        Trajectory dropSample = drive.trajectoryBuilder(resetBotPose)
                .splineToConstantHeading(new Vector2d(-25, 3.85), Math.toRadians(0))
                .build();

        Trajectory moveToSample = drive.trajectoryBuilder(resetBotPose)
                .splineToConstantHeading(new Vector2d(3, 0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(12, -6), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(10, -4), Math.toRadians(0))
                .build();

        Trajectory pickSample = drive.trajectoryBuilder(resetBotPose)
                .splineToConstantHeading(new Vector2d(0, -4), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(11.2, -4), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(11.2, -1.75), Math.toRadians(0))
                .build();

        Trajectory pickUpSample = drive.trajectoryBuilder(resetBotPose)
                .splineToConstantHeading(new Vector2d(5, 0), Math.toRadians(0))
                .build();


        waitForStart();

        if (isStopRequested()) return;


        drive.setPoseEstimate(resetBotPose);
        drive.followTrajectory(moveToBucket);

        drive.turn(Math.toRadians(-60));

        drive.setPoseEstimate(resetBotPose);
        drive.followTrajectory(goneToBucket);

        sampleslides(-5000);
        bucket_servo.setPosition(0.995);
        sleep(1000);
        bucket_servo.setPosition(0.25);
        outtake_slides_motor.setTargetPosition(-15);
        outtake_slides_motor.setPower(0.995);
        outtake_slides_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.setPoseEstimate(resetBotPose);
        drive.followTrajectory(moveToSample);

        drive.turn(Math.toRadians(60));

        intake_servo.setPower(1);
        intake_motor.setPower(0.995);
        sleep(200);
        intake_motor.setPower(-0.995);
        sleep(5);
        intake_motor.setPower(0.2);

        sleep(200);

        drive.setPoseEstimate(resetBotPose);
        drive.followTrajectory(pickSample);


        intake_servo.setPower(0.15);
        intake_motor.setPower(-0.2);
        drive.turn(Math.toRadians(15));

        sleep(50);

        intake_servo.setPower(0.995);
        drive.turn(Math.toRadians(17));

        sleep(50);

        intake_motor.setTargetPosition(0);
        intake_motor.setPower(0.8);
        intake_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(200);

        drive.setPoseEstimate(resetBotPose);
        drive.followTrajectory(pickUpSample);

        sleep(500);

        drive.setPoseEstimate(resetBotPose);
        drive.followTrajectory(goToBucket);

        intake_servo.setPower(-0.6);
        drive.turn(Math.toRadians(-77));

        drive.setPoseEstimate(resetBotPose);
        drive.followTrajectory(dropSample);

        sleep(200);

        sampleslides(-5000);
        bucket_servo.setPosition(0.995);
        sleep(1000);
        bucket_servo.setPosition(0.25);
        outtake_slides_motor.setTargetPosition(-15);
        outtake_slides_motor.setPower(0.995);
        outtake_slides_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(500);

    }

    public void sampleslides(int targetPosition) {

        String targetPositionNull_Debugger;

        targetPositionNull_Debugger = String.valueOf(targetPosition); //prevents null value from being set in setTargetPosition
        if (targetPositionNull_Debugger != null) {
            outtake_slides_motor.setTargetPosition(targetPosition);
            outtake_slides_motor.setPower(0.995);
            outtake_slides_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (outtake_slides_motor.isBusy()) {
            }
            sleep(270);
        }

    }
}