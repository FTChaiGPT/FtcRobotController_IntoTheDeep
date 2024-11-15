package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.RoadrunnerAdditionalActions.TurnFast;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



@Autonomous (name = "LM2SampleAuton", group = "A-LM2")

public class LM2_November13_SampleAuton extends LinearOpMode {

    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;

    private DcMotor outtake_slides_motor;
    private CRServo intake_servo;
    private Servo bucket_servo;
    private DcMotor intake_motor;
    private DcMotor hang_motor;
//    TurnFast robot = new TurnFast();
//    private VoltageSensor batteryVoltageSensor;
//    Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

    @Override
    public void runOpMode() {
        // batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");

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

//        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d resetBotPos = new Pose2d(0, 0, Math.toRadians(0));


        Trajectory Position1 = drive.trajectoryBuilder(startPose)
                .forward(3)
                .build();
        Trajectory Position2 = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(7, 24), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(5, 24), Math.toRadians(0))
                .build();
        Trajectory Position3 = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(-6, 0), Math.toRadians(0))
                .build();
        Trajectory SplinetoSample1 = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(11.4, 10), Math.toRadians(0))
                .build();
        Trajectory SplinetoSample2 = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(0, 10), Math.toRadians(0))
                .build();
        Trajectory PickSample11 = drive.trajectoryBuilder(resetBotPos)
                .forward(16.75)
                .build();
        Trajectory PickSample13 = drive.trajectoryBuilder(PickSample11.end())
                .forward(7)
                .build();
        Trajectory PickSample14 = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(73, 13), Math.toRadians(0))
                //.lineToSplineHeading(new Pose2d(72, 13, Math.toRadians(135)))
                .build();

        Trajectory PickSample12 = drive.trajectoryBuilder(resetBotPos)
                .back(2)
                .build();
        Trajectory gobackwards = drive.trajectoryBuilder(resetBotPos)
                .back(10)
                .build();


        waitForStart();

        if (isStopRequested()) return;
        //drop preload
        drive.setPoseEstimate(startPose);
        drive.followTrajectory(Position1);
        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(Position2);
        //robot.turnFast((long)-45.5, drive, batteryVoltageSensor, telemetry);
        drive.turn(Math.toRadians(-50));
        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(Position3);
        outtake_slides_motor.setPower(-0.995);
        sleep(1950);
        bucket_servo.setPosition(0.995);
        sleep(1000);
        drive.setPoseEstimate(resetBotPos);
        outtake_slides_motor.setPower(0.995);
        bucket_servo.setPosition(0.25);
        sleep(1600);
        //intake first sample
        intake_motor.setPower(-0.2);
        sleep(1500);
        intake_motor.setPower(0);
        drive.followTrajectory(SplinetoSample1);
        //robot.turnFast(35, drive, batteryVoltageSensor, telemetry);
        drive.turn(Math.toRadians(37));
        intake_servo.setPower(-0.995);
        sleep(1700);
        intake_motor.setPower(0.2);
        sleep(1500);
        intake_motor.setPower(0);
        intake_servo.setPower(0.995);
        sleep(1700);
        intake_servo.setPower(0);

        //robot.turnFast(-25, drive, batteryVoltageSensor, telemetry);
        drive.turn(Math.toRadians(-25));
        drive.followTrajectory(PickSample12);
        outtake_slides_motor.setPower(-0.995);
        sleep(1800);
        bucket_servo.setPosition(0.995);
        sleep(1000);
        drive.setPoseEstimate(resetBotPos);
        outtake_slides_motor.setPower(0.995);
        bucket_servo.setPosition(0.25);
        sleep(1700);
        //level 1 ascent
        drive.followTrajectory(PickSample14);
        drive.turn(Math.toRadians(135));
        //robot.turnFast(135, drive, batteryVoltageSensor, telemetry);

        hang_motor.setPower(-0.995);
        sleep(3000);
        hang_motor.setPower(0);
        hang_motor.setPower(0.995);
        sleep(3000);
        hang_motor.setPower(0);


    }
}
