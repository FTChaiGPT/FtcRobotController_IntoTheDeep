package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadrunnerAdditionalActions.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config //Can change FAST_TURN_FOR_SMALL_TURNS_AMOUNT in FTC Dashboard
@Autonomous(name = "TurnFastForSmallTurnsTest")
public class TurnFastForSmallTurnsTest extends LinearOpMode {

    public static long FAST_TURN_FOR_SMALL_TURNS_AMOUNT = 180;

    private VoltageSensor batteryVoltageSensor; //battery voltage

    private ElapsedTime timer = new ElapsedTime();
    private double timeStamp;

    private double previousBatteryVoltage;
    private double averageBatteryVoltage;

    @Override
    public void runOpMode() {

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        TurnFastForSmallTurns robot = new TurnFastForSmallTurns();

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        timer.reset();
        robot.turnFast(FAST_TURN_FOR_SMALL_TURNS_AMOUNT, drive, batteryVoltageSensor, telemetry);
        timeStamp = timer.milliseconds();

        telemetry.addData("Battery Voltage: ", batteryVoltageSensor.getVoltage());
        previousBatteryVoltage = batteryVoltageSensor.getVoltage();
        averageBatteryVoltage = (previousBatteryVoltage + batteryVoltageSensor.getVoltage()) / 2;

        while(opModeIsActive()) {
            telemetry.clearAll();
            telemetry.addData("TIME STAMP: ", timeStamp);
            drive.updatePoseEstimate();
            telemetry.addData("Current Pose: ", drive.getPoseEstimate());
            telemetry.addData("Battery Voltage: ", batteryVoltageSensor.getVoltage());
            if (previousBatteryVoltage != batteryVoltageSensor.getVoltage()) {
                averageBatteryVoltage = (averageBatteryVoltage + batteryVoltageSensor.getVoltage()) / 2;
            }
            previousBatteryVoltage = batteryVoltageSensor.getVoltage();
            telemetry.addData("Average Battery Voltage: ", averageBatteryVoltage);
            telemetry.update();
        }

    }
}
