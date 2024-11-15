package org.firstinspires.ftc.teamcode.RoadrunnerAdditionalActions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "FTCDashboardBatteryVoltageProvider")
public class FTCDashboardBatteryVoltageProvider extends LinearOpMode {
    /** FOR WHEN FTC DASHBOARD IS USED **/

    private VoltageSensor battery;

    private double previousBatteryVoltage;
    private double averageBatteryVoltage;

    @Override
    public void runOpMode() throws InterruptedException {

        battery = hardwareMap.voltageSensor.iterator().next();

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        telemetry.addData("Battery Voltage: ", battery.getVoltage());
        previousBatteryVoltage = battery.getVoltage();
        averageBatteryVoltage = (previousBatteryVoltage + battery.getVoltage()) / 2;

        while(opModeIsActive()) {
            telemetry.addData("Battery Voltage: ", battery.getVoltage());
            if (previousBatteryVoltage != battery.getVoltage()) {
                averageBatteryVoltage = (averageBatteryVoltage + battery.getVoltage()) / 2;
            }
            previousBatteryVoltage = battery.getVoltage();
            telemetry.addData("Average Battery Voltage: ", averageBatteryVoltage);
            telemetry.update();
        }

    }
}
