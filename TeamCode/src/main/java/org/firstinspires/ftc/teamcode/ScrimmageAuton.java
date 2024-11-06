package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "ScrimmageAuton")
public class ScrimmageAuton extends LinearOpMode {

    private DcMotor left_front;
    private DcMotor left_back;
    private DcMotor right_front;
    private DcMotor right_back;

    private DcMotor specimen_slides_motor;



    /* IMPORTANT INFO: Bot can't turn or spline, yes, severely disabled during auton, I'm on it but I don't know what I can do to fix it */

    @Override
    public void runOpMode() {

        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_back = hardwareMap.get(DcMotor.class, "right_back");

        specimen_slides_motor = hardwareMap.get(DcMotor.class, "specimen_slides_motor");
        specimen_slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // the SampleMecanumDrive class (RoadRunner) is made into an object


        Pose2d firstTurnPose = new Pose2d(9, -45.6, -90);

        Pose2d secondTurnPose = new Pose2d(47.6, -45.6, -180);

        Pose2d firstTurnWayBackPose = new Pose2d(47.6, -45.6, -90);

        Pose2d secondTurnScorePose = new Pose2d(13, -45.6, 0);

        Pose2d stageTwoPose = new Pose2d(8.75, -33, 0);
        Pose2d stageThreePose = new Pose2d(10.75, -33,0);

        Pose2d startPose = new Pose2d(9, -63, 0);
        drive.setPoseEstimate(startPose);

        Trajectory goToRung = drive.trajectoryBuilder(startPose)
                .forward(34.4)
                .build();


        Trajectory prepForTurn = drive.trajectoryBuilder(stageTwoPose)
                .back(17)
                .build();

        Trajectory prepForNextTurn = drive.trajectoryBuilder(stageThreePose)
                .back(17)
                .build();

        Trajectory horizAlignWithSpecimen = drive.trajectoryBuilder(firstTurnPose)
                .forward(38.6)
                .build();

        Trajectory horizWayBackWithSpecimen = drive.trajectoryBuilder(firstTurnWayBackPose)
                .back(34)
                .build();

        Trajectory vertAlignWithSpecimen = drive.trajectoryBuilder(secondTurnPose)
                .forward(23.25)
                .build();


        Trajectory goBackWithSpecimen = drive.trajectoryBuilder(secondTurnPose)
                .back(8)
                .build();

        Trajectory SecondTurnScoreSpecimen = drive.trajectoryBuilder(secondTurnScorePose)
                .forward(12.5)
                .build();


        waitForStart();


        if (isStopRequested()) return;

        specimen_slides_motor.setPower(0.5);
        drive.followTrajectory(goToRung);
        specimen_slides_motor.setPower(0);

        specimen_slides_motor.setPower(-0.995);
        sleep(100);
        drive.setPoseEstimate(stageTwoPose);
        drive.followTrajectory(prepForTurn);
        specimen_slides_motor.setPower(0);

        drive.turn(Math.toRadians(-92));

        drive.setPoseEstimate(firstTurnPose);
        drive.followTrajectory(horizAlignWithSpecimen);

        drive.turn(Math.toRadians(-92));

        drive.setPoseEstimate(secondTurnPose);
        drive.followTrajectory(vertAlignWithSpecimen);

        sleep(100);

        specimen_slides_motor.setPower(0.995);
        sleep(310);
        specimen_slides_motor.setPower(0.6);
        sleep(930);
        specimen_slides_motor.setPower(0.175);

        drive.followTrajectory(goBackWithSpecimen);

        drive.turn(Math.toRadians(92));

        drive.setPoseEstimate(firstTurnWayBackPose);
        drive.followTrajectory(horizWayBackWithSpecimen);

        drive.turn(Math.toRadians(94));

        drive.setPoseEstimate(secondTurnScorePose);
        drive.followTrajectory(SecondTurnScoreSpecimen);
        specimen_slides_motor.setPower(0);

        specimen_slides_motor.setPower(-0.995);
        sleep(350);


        left_front.setPower(-0.995);
        left_back.setPower(-0.995);
        right_front.setPower(-0.225);
        right_back.setPower(-0.225);
        specimen_slides_motor.setPower(0);
        sleep(950);
        left_front.setPower(-0.995);
        left_back.setPower(-0.995);
        right_front.setPower(-0.995);
        right_back.setPower(-0.995);
        sleep(300);
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);



        while (opModeIsActive()) {
            telemetry.addLine("done :D");
            telemetry.update();
        }
    }
}
