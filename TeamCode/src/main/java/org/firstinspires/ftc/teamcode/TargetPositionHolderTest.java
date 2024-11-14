package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.DriveAdditionalActions.*;

@Config
@Autonomous (name = "TargetPositionHolderTest")
public class TargetPositionHolderTest extends OpMode {

    private VoltageSensor batteryVoltageSensor;
    private TargetPositionHolder robot = new TargetPositionHolder();

    public static long targetPosition = -4000;

    private DcMotor motor;

    private static double TICK_PER_REV = 537.7;

    @Override
    public void init() {

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        motor = hardwareMap.get(DcMotor.class, "outtake_slides_motor");

        telemetry.addLine("On Gamepad1, a or b to hold.");
        telemetry.addLine("    ~ b shows use of varargs (optional fields), in code.");
        telemetry.addLine(" ");
        telemetry.addLine("On Gamepad1, y to release hold");
        telemetry.addLine("On Gamepad1, x to show other things can run while motor holds.");
        telemetry.update();


    }

    @Override
    public void loop() {
        if (gamepad1.a) robot.holdDcMotor(motor, targetPosition, batteryVoltageSensor, "ALLOWABLE_MARGIN_OF_ERROR", 1);

        if (gamepad1.b) robot.holdDcMotor(motor, targetPosition, batteryVoltageSensor, "TICKS_PER_REV", TICK_PER_REV, "ALLOWABLE_MARGIN_OF_ERROR", 5);

        if (gamepad1.y) {
            robot.stopHoldingDcMotor(motor);
        }

        commentary();
    }


    @Override//                                            _
    public void stop() {//                                  |
        robot.killExecutor(motor, batteryVoltageSensor);//  |`---- ⚠ Failing to add this chunk of code will result in spontaneous
    }//                                                    _|    holding of motor(s) upon initialization and will require turning
     //                                                          the bot off & on or re-downloading. ⚠

    public void commentary() {
        if (gamepad1.x) {
            telemetry.addLine("Holding code uses ExecutorService and thus runs async");
            telemetry.addLine("you can do other things such as printing comments");
            telemetry.addLine("and running actions.");
            telemetry.update();
        }
        else {
            telemetry.addLine("On Gamepad1, a or b to hold.");
            telemetry.addLine("    ~ b shows use of varargs (optional fields), in code.");
            telemetry.addLine(" ");
            telemetry.addLine("On Gamepad1, y to release hold");
            telemetry.addLine("On Gamepad1, x to show other things can run while motor holds.");
            telemetry.update();
        }
    }


}