package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ClawMechanism")

public class ClawMechanism extends OpMode {

    private Servo claw_servo;
    private CRServo clawPivot_crservo;

    private boolean clawToggle = true;
    private boolean prev_gamepad2a = false;
    private boolean cur_gamepad2a = false;

    @Override
    public void init() {
        claw_servo = hardwareMap.servo.get("claw_servo");
        clawPivot_crservo = hardwareMap.crservo.get("claw_pivot");

    }

    @Override
    public void loop() {
        if (gamepad2.b) {
            clawPivot_crservo.setPower(0.5);
        }
        else {
            clawPivot_crservo.setPower(0);
        }
        if (gamepad2.x){
            clawPivot_crservo.setPower(-0.5);
        }
        else{
            clawPivot_crservo.setPower(0);
        }

        prev_gamepad2a = cur_gamepad2a;
        cur_gamepad2a = gamepad2.a;

        if (cur_gamepad2a && !prev_gamepad2a) {
            if (clawToggle) { //clawToggle is equal true
                clawToggle = false;
                claw_servo.setPosition(0);
            }
            else {
                clawToggle = true;
                claw_servo.setPosition(1);
            }
        }
    }
}