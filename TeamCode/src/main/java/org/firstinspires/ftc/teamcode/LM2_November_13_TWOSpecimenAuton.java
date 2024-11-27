package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//@Disabled
@Autonomous (name = "LM2_November_13_TWOSpecimenAuton", group = "A_LM1")
public class LM2_November_13_TWOSpecimenAuton extends LinearOpMode {

    private DcMotorEx left_front;
    private DcMotorEx left_back;
    private DcMotorEx right_front;
    private DcMotorEx right_back;

    private DcMotor specimen_slides_motor;

    @Override
    public void runOpMode() {

        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        right_back = hardwareMap.get(DcMotorEx.class, "right_back");

        specimen_slides_motor = hardwareMap.get(DcMotor.class, "specimen_slides_motor");
        specimen_slides_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // the SampleMecanumDrive class (RoadRunner) is made into an object
        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Pose2d startPose = new Pose2d(9, -63, 0);

        Pose2d resetBotPos = new Pose2d(0, 0, 0);

        Pose2d pickSpecimenPose = new Pose2d(0, 0, Math.toRadians(0));

        Trajectory goToRung = drive.trajectoryBuilder(startPose)
                .forward(34.5)
                .build();

        Trajectory releaseClinchWithRung = drive.trajectoryBuilder(goToRung.end())
                .back(8)
                .build();

        Trajectory moveBehindSamples = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(0,-23), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(16,-24), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(33,-38), Math.toRadians(0))
                .build();

        Trajectory moveSamples = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(48,0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45,0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45,-4), Math.toRadians(0))
                .build();

        Trajectory pickSpecimenOne = drive.trajectoryBuilder(pickSpecimenPose)
                .back(-24.5)
                .build();

        Trajectory lilRealeaseClinchWithWall_PART_ONE = drive.trajectoryBuilder(resetBotPos)
                .forward(-2)
                .build();

        Trajectory lilRealeaseClinchWithWall_PART_TWO = drive.trajectoryBuilder(resetBotPos)
                .forward(1.1)
                .build();

        Trajectory reClinchWithWall = drive.trajectoryBuilder(resetBotPos)
                .forward(2)
                .build();

        Trajectory releaseClinchWithWall = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(-8,0), Math.toRadians(0))
                .build();

        Trajectory goBackWithSpecimenTwo = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(18,28), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(22.685,28), Math.toRadians(0))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(startPose);

        specimen_slides_motor.setPower(0.55);
        drive.followTrajectory(goToRung);
        specimen_slides_motor.setPower(0);

        specimen_slides_motor.setPower(-0.995);
        sleep(300);
        drive.followTrajectory(releaseClinchWithRung);
        specimen_slides_motor.setPower(0);

        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(moveBehindSamples);
        drive.turn(Math.toRadians(186));
        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(moveSamples);

        sleep(20);

        drive.setPoseEstimate(pickSpecimenPose);
        drive.followTrajectory(pickSpecimenOne);
        specimen_slides_motor.setPower(0.2);
        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(lilRealeaseClinchWithWall_PART_ONE);
        sleep(85);
        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(lilRealeaseClinchWithWall_PART_TWO);
        specimen_slides_motor.setPower(0.75);
        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(reClinchWithWall);
        specimen_slides_motor.setPower(0.35);

        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(releaseClinchWithWall);
        drive.turn(Math.toRadians(180));
        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(goBackWithSpecimenTwo);

        specimen_slides_motor.setPower(-0.995);
        sleep(425);

        left_front.setPower(-0.995);
        left_back.setPower(-0.995);
        right_front.setPower(-0.2);
        right_back.setPower(-0.2);
        sleep(400);
        specimen_slides_motor.setPower(0);
        sleep(425);
        left_front.setPower(-0.995);
        left_back.setPower(-0.995);
        right_front.setPower(-0.995);
        right_back.setPower(-0.995);
        sleep(150);
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);

    }
}

