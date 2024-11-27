package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "ZbinZerboVone")
public class ZpinZervoOne extends OpMode {

    private Servo servo;

    public static double ONE_TURN_CONSTANT = 0.2155;
    public static double TURN_AMOUNT = 90;
    public static double TIME_CONSTANT = 5500;


    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
    }

    @NonNull
    @Override
    public void loop() {

        servo.setPosition(0);

        sleep((long)TIME_CONSTANT);

        telemetry.addData("servo inputed value: ", translateToFiveTurnServoPosition(TURN_AMOUNT));
        servo.setPosition(translateToFiveTurnServoPosition(TURN_AMOUNT));
        telemetry.update();

        sleep((long)TIME_CONSTANT);
    }

    @NonNull
    public double translateToFiveTurnServoPosition(double degrees) {
        return (degrees / 360) * ONE_TURN_CONSTANT;
    }

}
