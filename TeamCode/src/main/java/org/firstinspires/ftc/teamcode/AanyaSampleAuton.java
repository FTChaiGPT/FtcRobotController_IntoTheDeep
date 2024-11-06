package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "LM1_October_26_SampleAut66on", group = "LM1_October_26")

public class AanyaSampleAuton extends LinearOpMode {

    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;

    private DcMotor outtake_slides_motor;
    private CRServo intake_servo;
    private Servo bucket_servo;
    private DcMotor intake_motor;


    @Override
    public void runOpMode() {

        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");

        outtake_slides_motor = hardwareMap.get(DcMotor.class, "specimen_slides_motor");
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


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -63, 0);
        Pose2d Pose2 = new Pose2d(-47, -41, -20);
        Pose2d resetBotPos = new Pose2d(0, 0, 0);
        Pose2d Pose3 = new Pose2d(-47, -36, 50);

        Pose2d dropSamplePose = new Pose2d(-62, -60, Math.toRadians(0));


        Trajectory gofromStart = drive.trajectoryBuilder(startPose)
                .forward(24.5)
                .build();


        Trajectory PickUpSample1 = drive.trajectoryBuilder(Pose2)
                .forward(2)
                .build();

        Trajectory GoToBucket = drive.trajectoryBuilder(Pose3)
                .splineTo(new Vector2d(-60, -52), Math.toRadians(240))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(startPose);
        drive.followTrajectory(gofromStart);
        drive.turn(Math.toRadians(50));
        drive.setPoseEstimate(Pose2);
        drive.followTrajectory(PickUpSample1);
        drive.setPoseEstimate(Pose3);
        intake_motor.setPower(-0.995);
        sleep(1200);
        intake_motor.setPower(0);
        intake_servo.setPower(-0.995);
        sleep(3000);
        intake_motor.setPower(0.995);
        drive.followTrajectory(GoToBucket);
        intake_motor.setPower(-0.995);
        outtake_slides_motor.setPower(0.995);
        sleep(2000);
        bucket_servo.setPosition(0.995);
        sleep(1000);
        bucket_servo.setPosition(0);
        outtake_slides_motor.setPower(-0.995);
        sleep(2000);

        //intake_motor.setTargetPosition(0);
        //intake_servo.setPower(-0.995);
        //sleep(1000);
        //drive.followTrajectory(dropSamples);
        //outtake_slides_motor.setTargetPosition(-4874);
        //bucket_servo.setPosition(0.995);
        //sleep(1000);
        //outtake_slides_motor.setTargetPosition(20);
        //bucket_servo.setPosition(0.995);

        //outtake_slides_motor.setPower(0);

        //drive.setPoseEstimate(resetBotPos);
        //drive.followTrajectory(moveBehindSamples);

        //drive.setPoseEstimate(resetBotPos);
        //drive.followTrajectory(moveSamples);

        //sleep(100);

    }
}

