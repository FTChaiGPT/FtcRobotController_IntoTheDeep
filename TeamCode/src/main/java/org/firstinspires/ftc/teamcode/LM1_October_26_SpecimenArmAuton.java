package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "LM1_October_26_SpecimenArmAuton", group = "A_LM1")

public class LM1_October_26_SpecimenArmAuton extends LinearOpMode {

    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;

    private DcMotor specimen_slides_motor;
    private DcMotor intake_motor;

    private CRServo intake_servo;

    private double SPLINE_X_MULTIPLIER = 1.022818745 / 3;
    private double SPLINE_Y_MULTIPLIER = 1.19062265 / 3;

    @Override
    public void runOpMode() {

        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");

        specimen_slides_motor = hardwareMap.get(DcMotor.class, "specimen_slides_motor");
        specimen_slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake_servo = hardwareMap.get(CRServo.class, "intake_servo");
        intake_servo.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // the SampleMecanumDrive class (RoadRunner) is made into an object

        Pose2d startPose = new Pose2d(9, -63, 0);

        Pose2d resetBotPos = new Pose2d(0, 0, 0);

        Pose2d pickSpecimenPose = new Pose2d(0, 0, Math.toRadians(0));

        Trajectory goToRung = drive.trajectoryBuilder(startPose)
                .forward(34.7)
                .build();

        Trajectory releaseClinchWithRung = drive.trajectoryBuilder(goToRung.end())
                .back(16)
                .build();

        Trajectory moveSampleStep1 = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(30,0), Math.toRadians(0))
                .build();

        Trajectory alignToSpecimen = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(-5,-3), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(22,0), Math.toRadians(0))
                .build();

        Trajectory moveSampleStep2 = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(18,10), Math.toRadians(0))
                .build();

        Trajectory pickSpecimenOne = drive.trajectoryBuilder(pickSpecimenPose)
                .back(-20)
                .build();

        Trajectory goBackWithSpecimenTwo = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(-24,0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-24,-34), Math.toRadians(0))
                .build();

        Trajectory scoreSpecimenTwo = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(10,0), Math.toRadians(0))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(startPose);

        specimen_slides_motor.setPower(0.47);
        drive.followTrajectory(goToRung);
        specimen_slides_motor.setPower(0);

        specimen_slides_motor.setPower(-0.995);
        sleep(220);
        drive.followTrajectory(releaseClinchWithRung);
        specimen_slides_motor.setPower(0);
        drive.turn(Math.toRadians(-66.8));

        intake_servo.setPower(0.995);
        intake_motor.setTargetPosition(-176);
        intake_motor.setPower(0.5);
        intake_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(100);

        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(moveSampleStep1);
        drive.turn(Math.toRadians(-113.2));

        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(moveSampleStep2);

        intake_servo.setPower(-0.995);
        sleep(150);

        intake_motor.setTargetPosition(0);
        intake_motor.setPower(0.5);
        intake_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake_servo.setPower(0);

        sleep(850);

        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(alignToSpecimen);

        specimen_slides_motor.setPower(0.995);
        sleep(150);
        specimen_slides_motor.setPower(0.35);

        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(goBackWithSpecimenTwo);
        drive.turn(Math.toRadians(184));

        specimen_slides_motor.setPower(-0.25);
        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(scoreSpecimenTwo);
        specimen_slides_motor.setPower(-0.995);
        sleep(90);
        specimen_slides_motor.setPower(0);

        left_front.setPower(-0.995);
        left_back.setPower(-0.995);
        right_front.setPower(-0.2335);
        right_back.setPower(-0.2335);
        specimen_slides_motor.setPower(0);
        sleep(900);
        left_front.setPower(-0.995);
        left_back.setPower(-0.995);
        right_front.setPower(-0.995);
        right_back.setPower(-0.995);
        sleep(100);
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
    }
}
