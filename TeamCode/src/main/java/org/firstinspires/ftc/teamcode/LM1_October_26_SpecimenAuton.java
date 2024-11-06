package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "LM1_October_26_SpecimenAuton", group = "A_LM1")
public class LM1_October_26_SpecimenAuton extends LinearOpMode {

    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;

    private DcMotor specimen_slides_motor;

    @Override
    public void runOpMode() {

        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");

        specimen_slides_motor = hardwareMap.get(DcMotor.class, "specimen_slides_motor");
        specimen_slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // the SampleMecanumDrive class (RoadRunner) is made into an object
        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Pose2d startPose = new Pose2d(9, -63, 0);

        Pose2d resetBotPos = new Pose2d(0, 0, 0);

        Pose2d pickSpecimenPose = new Pose2d(0, 0, Math.toRadians(0));

        Trajectory goToRung = drive.trajectoryBuilder(startPose)
                .forward(32)
                .build();

        Trajectory releaseClinchWithRung = drive.trajectoryBuilder(goToRung.end())
                .back(8)
                .build();

        Trajectory moveBehindSamples = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(0,-24), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(16,-24), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(33,-35), Math.toRadians(0))
                .build();

        Trajectory moveSamples = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(48,0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45,0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45,-4.25), Math.toRadians(0))
                .build();

        Trajectory pickSpecimenOne = drive.trajectoryBuilder(pickSpecimenPose)
                .back(-24.5)
                .build();

        Trajectory releaseClinchWithWall = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(-8,0), Math.toRadians(0))
                .build();

        Trajectory goBackWithSpecimenTwo = drive.trajectoryBuilder(releaseClinchWithWall.end())
                .splineToLinearHeading(new Pose2d(-3,-35, Math.toRadians(180)), Math.toRadians(0))
                .build();

        Trajectory scoreSpecimenTwo = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(2,0), Math.toRadians(0))
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

        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(moveBehindSamples);
        drive.turn(Math.toRadians(186));
        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(moveSamples);

        sleep(100);

        drive.setPoseEstimate(pickSpecimenPose);
        drive.followTrajectory(pickSpecimenOne);
        specimen_slides_motor.setPower(0.995);
        sleep(50);
        specimen_slides_motor.setPower(0.35);

        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(releaseClinchWithWall);
        drive.followTrajectory(goBackWithSpecimenTwo);

        specimen_slides_motor.setPower(-0.2);
        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(scoreSpecimenTwo);
        specimen_slides_motor.setPower(-0.995);
        sleep(145);
        specimen_slides_motor.setPower(0);

        left_front.setPower(-0.995);
        left_back.setPower(-0.995);
        right_front.setPower(-0.2235);
        right_back.setPower(-0.2235);
        specimen_slides_motor.setPower(0);
        sleep(900);
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

