package org.firstinspires.ftc.teamcode.RoadrunnerAdditionalActions;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TurnQuickly extends LinearOpMode {

    private Pose2d startPose = new Pose2d(0,0,0);

    public void turnQuickly180(SampleMecanumDrive drive) {

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Pose2d currentPose = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading());
        drive.setPoseEstimate(currentPose);

        Trajectory trajEnd = drive.trajectoryBuilder(currentPose)
                .splineToConstantHeading(new Vector2d(0,0), drive.getPoseEstimate().getHeading())
                .build();

        Trajectory traj = drive.trajectoryBuilder(currentPose)
                .splineToConstantHeading(new Vector2d(-0.001,-0.001), drive.getPoseEstimate().getHeading())
                .splineToConstantHeading(new Vector2d(0.002,0.002), drive.getPoseEstimate().getHeading())
                .build();

        int n = 2;
        for (int i = 0; i < n; i++) {
            drive.updatePoseEstimate();
            drive.followTrajectory(traj);
        }
        drive.followTrajectory(trajEnd);
    }

    public void runOpMode() {}
}
