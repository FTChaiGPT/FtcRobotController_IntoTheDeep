package org.firstinspires.ftc.teamcode.RoadrunnerAdditionalActions;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp (name = "BatteryDrainer") //USE FOR GETTING BATTERY LOW FOR TURNFAST TUNING
public class BatteryDrainer extends OpMode {

    public static double BATTERY_DRAIN_DISTANCE = 60;
    public static double BATTERY_DRAIN_SESSION = 60000;
    public static double BATTERY_DRAIN_REST_TIME = 100000;
    ElapsedTime timer = new ElapsedTime();

    private Pose2d PoseFirst = new Pose2d(0,0,0);
    private Pose2d PoseLast = new Pose2d(0, BATTERY_DRAIN_DISTANCE, 0);
    private int driveIterator = 0;

    @Override
    public void init() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        drive.setPoseEstimate(PoseFirst);
    }


    @Override
    public void loop() {

        timer.reset();

        telemetry.clearAll();

        while (timer.milliseconds() <= BATTERY_DRAIN_SESSION) {
            telemetry.addLine("Draining");
            telemetry.update();

        }

        timer.reset();

        telemetry.clearAll();

        while (timer.milliseconds() <= BATTERY_DRAIN_REST_TIME) {
            telemetry.addLine("Resting");
            telemetry.update();
        }



    }

    public void drain(SampleMecanumDrive drive) {
        if (driveIterator == 0) {
            drive.setPoseEstimate(PoseFirst);
            driveIterator++;
            Trajectory trajStrafeLeft = drive.trajectoryBuilder(PoseFirst).splineToConstantHeading(new Vector2d(0, BATTERY_DRAIN_DISTANCE),0).build();
        }
        else {
            drive.setPoseEstimate(PoseLast);
            driveIterator--;
            Trajectory trajStrafeRight = drive.trajectoryBuilder(PoseLast).splineToConstantHeading(new Vector2d(0, 0),0).build();
        }
    }

}
