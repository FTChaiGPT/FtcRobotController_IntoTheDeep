package org.firstinspires.ftc.teamcode.RoadrunnerAdditionalActions;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
//Can edit TURN_AMT
@Autonomous(name = "TurnFastForSmallTurns")
public class TurnFastForSmallTurns extends LinearOpMode {

    private int turnSectionIterator = 0;

    public static double TURN_AMT = 45;

    public void turnFast(double smallFastTurnAmount, SampleMecanumDrive drive, VoltageSensor batteryVoltageSensor, Telemetry telemetry) {

        if (drive == null || batteryVoltageSensor == null || smallFastTurnAmount == 0) return;

        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        /** drive = new SampleMecanumDrive(hardwareMap); is not applied because that parameter is passed in when the method is used if the object is created in this class itself, it will not allow itself to be used other than as a null object and then will lead to error: NullPointerException, cannot call "{whatever}" on null object reference. **/

        Pose2d SSTTcurrentPose = new Pose2d(0,0,0);
        drive.setPoseEstimate(SSTTcurrentPose);

        telemetry.addLine("Turning");
        telemetry.update();

        double turnRequirement = (batteryVoltageSensor.getVoltage() > 13) ? (((smallFastTurnAmount / 45) < 1) ? 1 : (smallFastTurnAmount / 45)) : (((smallFastTurnAmount / 45) < 1) ? 1 : (smallFastTurnAmount / ((20/45) * TURN_AMT)));
        for (double i = 0; i <= turnRequirement; i++) { //run 1 extra time
            turnInSection(smallFastTurnAmount, drive, batteryVoltageSensor);
        }

        Pose2d SSTTendPose = drive.getPoseEstimate();
        drive.updatePoseEstimate();
        if (drive.getPoseEstimate() != SSTTcurrentPose) { //spline adjustment
            Trajectory fixPose = drive.trajectoryBuilder(drive.getPoseEstimate()).splineToConstantHeading(new Vector2d(SSTTcurrentPose.getX(), SSTTcurrentPose.getY()), drive.getPoseEstimate().getHeading()).build();
            drive.followTrajectoryAsync(fixPose);
        }
        drive.setPoseEstimate(SSTTendPose);
        telemetry.addLine("Small Fast Turn Completed!");
        telemetry.update();
        drive.waitForIdle();
        telemetry.addLine("Drive Idle.");
        telemetry.update();
    }

    public void turnInSection(double smallFastTurnAmount, SampleMecanumDrive drive, VoltageSensor batteryVoltageSensor) {
        drive.setPoseEstimate(drive.getPoseEstimate());
        turnSectionIterator++;
        if (Math.ceil(turnSectionIterator) == turnSectionIterator) {
            if (smallFastTurnAmount > 0) {
               turnSection2(smallFastTurnAmount, drive, batteryVoltageSensor);
            }
            else {
                turnSection1(smallFastTurnAmount, drive, batteryVoltageSensor);
            }
        }
        else {
            if (smallFastTurnAmount > 0) {
                turnSection1(smallFastTurnAmount, drive, batteryVoltageSensor);
            }
            else {
                turnSection2(smallFastTurnAmount, drive, batteryVoltageSensor);
            }
        }
    }

    public void turnSection1(double smallFastTurnAmount, SampleMecanumDrive drive, VoltageSensor batteryVoltageSensor) {
        Trajectory turnTraj = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()).plus(new Pose2d(0, 0, Math.toRadians(45))))
                .splineToConstantHeading(new Vector2d(0.003, 0.003), Math.toRadians(0)).build();
    }

    public void turnSection2(double smallFastTurnAmount, SampleMecanumDrive drive, VoltageSensor batteryVoltageSensor) {
        Trajectory turnTraj = drive.trajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()).plus(new Pose2d(0, 0, Math.toRadians(45))))
                .splineToConstantHeading(new Vector2d(-0.003, -0.003), Math.toRadians(0)).build();
    }


    public void runOpMode() {/* --EMPTY-- */}
}
