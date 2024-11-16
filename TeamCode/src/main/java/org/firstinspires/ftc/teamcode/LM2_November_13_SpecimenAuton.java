package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.RoadrunnerAdditionalActions.*;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "LM2_November_13_SpecimenAuton", group = "A_LM1")
public class LM2_November_13_SpecimenAuton extends LinearOpMode {

    private DcMotorEx left_front;
    private DcMotorEx left_back;
    private DcMotorEx right_front;
    private DcMotorEx right_back;

    private VoltageSensor batteryVoltageSensor;

    private DcMotor specimen_slides_motor;

    @Override
    public void runOpMode() {

        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        right_back = hardwareMap.get(DcMotorEx.class, "right_back");

        specimen_slides_motor = hardwareMap.get(DcMotor.class, "specimen_slides_motor");
        specimen_slides_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // the SampleMecanumDrive class (RoadRunner) is made into an object
        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        TurnFast robot = new TurnFast();

        Pose2d startPose = new Pose2d(9, -63, 0);

        Pose2d resetBotPos = new Pose2d(0, 0, 0);

        Pose2d pickSpecimenPose = new Pose2d(0, 0, Math.toRadians(0));

        Trajectory goToRung = drive.trajectoryBuilder(startPose)
                .forward(33.3)
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
                .splineToConstantHeading(new Vector2d(45,-8), Math.toRadians(0))
                .build();

        Trajectory pickSpecimenTwo = drive.trajectoryBuilder(pickSpecimenPose)
                .back(-24.5)
                .build();

        Trajectory pickSpecimenThree = drive.trajectoryBuilder(pickSpecimenPose)
                .back(-10)
                .build();

        Trajectory lilRealeaseClinchWithWall = drive.trajectoryBuilder(pickSpecimenTwo.end())
                .back(3)
                .build();

        Trajectory laterLilRealeaseClinchWithWall = drive.trajectoryBuilder(pickSpecimenThree.end())
                .back(3)
                .build();

        Trajectory releaseClinchWithWall = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(-8,0), Math.toRadians(0))
                .build();

        Trajectory goBackWithSpecimenTwo = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(18,24), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(24,24), Math.toRadians(0))
                .build();

        Trajectory goToWallForThirdSpecimen = drive.trajectoryBuilder(resetBotPos)
                .splineToConstantHeading(new Vector2d(-8,0), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-8,-23), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-30,-29.25), Math.toRadians(0))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(startPose);

        specimen_slides_motor.setPower(0.55);
        drive.followTrajectory(goToRung);
        specimen_slides_motor.setPower(0);

        specimen_slides_motor.setPower(-0.995);
        sleep(200);
        drive.followTrajectory(releaseClinchWithRung);
        specimen_slides_motor.setPower(0);

        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(moveBehindSamples);

        sleep(100);
        robot.turnFast(180, drive, batteryVoltageSensor, telemetry);
        sleep(280);

        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(moveSamples);

        sleep(30);

        drive.setPoseEstimate(pickSpecimenPose);
        drive.followTrajectory(pickSpecimenTwo);
        specimen_slides_motor.setPower(0.35);
        drive.followTrajectory(lilRealeaseClinchWithWall);
        specimen_slides_motor.setPower(0.995);
        sleep(70);
        specimen_slides_motor.setPower(0.35);

        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(releaseClinchWithWall);

        sleep(100);
        robot.turnFast(180, drive, batteryVoltageSensor, telemetry);
        sleep(280);

        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(goBackWithSpecimenTwo);

        specimen_slides_motor.setPower(-0.995);
        sleep(275);
        specimen_slides_motor.setPower(0);

        /** where 3rd spec starts **/
        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(goToWallForThirdSpecimen);

        sleep(150);
        robot.turnFast(180, drive, batteryVoltageSensor, telemetry);
        sleep(300);

        drive.setPoseEstimate(pickSpecimenPose);
        drive.followTrajectory(pickSpecimenThree);
        specimen_slides_motor.setPower(0.35);
        drive.followTrajectory(laterLilRealeaseClinchWithWall);
        specimen_slides_motor.setPower(0.995);
        sleep(70);
        specimen_slides_motor.setPower(0.35);

        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(releaseClinchWithWall);

        sleep(100);
        robot.turnFast(180, drive, batteryVoltageSensor, telemetry);
        sleep(300);

        drive.setPoseEstimate(resetBotPos);
        drive.followTrajectory(goBackWithSpecimenTwo);

        specimen_slides_motor.setPower(-0.995);
        sleep(275);
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

