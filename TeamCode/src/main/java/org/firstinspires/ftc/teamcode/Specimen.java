package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name="Specimen", group = "specimen")
public class Specimen extends OpMode {
    private DcMotor slides_motor;


    @Override
    public void init() {
        slides_motor = hardwareMap.get(DcMotor.class, "slides_motor");
        slides_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void slides() {
        if (gamepad2.left_stick_y > 0.1) {
            //move slides up
            slides_motor.setPower(-0.6);
        } else if (gamepad2.left_stick_y < -0.1) {
            //move slides down
            slides_motor.setPower(0.6);
        } else {

            slides_motor.setPower(0);
        }
    }




    @Override
    public void loop() {
        slides();


        telemetry.update();
    }
}