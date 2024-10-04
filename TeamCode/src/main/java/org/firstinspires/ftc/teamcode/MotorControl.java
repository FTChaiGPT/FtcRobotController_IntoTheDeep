package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * This class controls the motor based on the detected color.
 * If the detected color is blue, the motor runs.
 */

@TeleOp(name = "ColorSensorTest")
public class MotorControl extends LinearOpMode {

    private OurColorSensor ourColorSensor;

    private OurColorSensor.DetectedColor detectedColor;

    @Override
    public void runOpMode() throws InterruptedException {
        ourColorSensor = new OurColorSensor();
        ourColorSensor.init(hardwareMap);


        waitForStart();

        while(opModeIsActive()){
            detectedColor = ourColorSensor.detectColor(telemetry);

            telemetry.addData("Detected Color", detectedColor);
            telemetry.update();

            if (detectedColor.equals(OurColorSensor.DetectedColor.BLUE)){
                telemetry.addData("Intake Sample", "");
                telemetry.update();
            }
        }
    }


}

