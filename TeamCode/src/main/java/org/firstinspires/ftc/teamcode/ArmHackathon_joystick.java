package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Arm Encoder Control", group="TeleOp")
public class ArmHackathon_joystick extends OpMode {
    private DcMotor slides_motor;
    private DcMotor slidesPivot_motor;
    private int left_targetPosition = 0;
    private int right_targetPosition = 0;

    @Override
    public void init() {
        // Initialize motor
        slides_motor = hardwareMap.get(DcMotor.class, "slides_motor");
        slidesPivot_motor = hardwareMap.get(DcMotor.class, "slidespivot_motor");

        // Set motor to brake when no power is applied
        slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reset encoder
        slides_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesPivot_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Get joystick input
        double left_joystickY = -gamepad1.left_stick_y;
        double right_joystickY = -gamepad1.right_stick_y;

        if (Math.abs(left_joystickY) > 0.1) {
            // Adjust target position based on joystick input
            left_targetPosition += left_joystickY * 20;  // Change multiplier based on arm speed preference
        }

        if (Math.abs(right_joystickY) > 0.1) {
            // Adjust target position based on joystick input
            right_targetPosition += right_joystickY * 20;  // Change multiplier based on arm speed preference
        }

        // Set target position for the motor
        slides_motor.setTargetPosition(left_targetPosition);
        slidesPivot_motor.setTargetPosition(right_targetPosition);


        // Switch to RUN_TO_POSITION mode
        slides_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesPivot_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply a fixed power to move towards the target position
        slides_motor.setPower(0.5);
        slidesPivot_motor.setPower(0.5);// You can adjust power based on how fast you want the arm to move

        // Telemetry for debugging
        telemetry.addData("Target Position", left_targetPosition);
        telemetry.addData("Current Position", slides_motor.getCurrentPosition());
        telemetry.addData("Motor is at target", !slides_motor.isBusy());
        telemetry.update();
    }
}