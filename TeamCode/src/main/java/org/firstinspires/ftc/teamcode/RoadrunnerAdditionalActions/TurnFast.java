package org.firstinspires.ftc.teamcode.RoadrunnerAdditionalActions;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Config //Can edit FAST_TURN_WAIT_TIME, OPTIMAL_BATTERY_VOLTAGEs, and TRAINED_EXP_MULTIPLIERs in FTC Dashboard
@Autonomous (name = "TurnFast")
public class TurnFast extends LinearOpMode {

    // TODO: Tune MAX_VEL ~ DONE
    // TODO: Tune MAX_ACCEL ~ DONE
    // TODO: Tune/Check MAX_ANG_ACCEL ~ DONE - usually already perfect

    public static double FAST_TURN_BASE_WAIT_TIME = 630;
    public static double TUNING_TURN_ANGLE = 180;
    public static double BASE_SCALE_FACTOR = 1.0;
    //public static double SCALE_FACTOR2 = 0.88;
    public static double SCALE_FACTOR2 = 0.55;

    public static double OPTIMAL_BATTERY_VOLTAGE = 12.7321;
    public static double OPTIMAL_BATTERY_VOLTAGE2 = 13.7335;

    private ElapsedTime timer = new ElapsedTime();

    public void turnFast(double fastTurnAmount, SampleMecanumDrive drive, VoltageSensor batteryVoltageSensor, Telemetry telemetry) {

        if (drive == null || batteryVoltageSensor == null || fastTurnAmount == 0) return;

        double FAST_TURN_WAIT_TIME = (FAST_TURN_BASE_WAIT_TIME / TUNING_TURN_ANGLE) * fastTurnAmount;

        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            /** drive = new SampleMecanumDrive(hardwareMap); is not applied because that parameter is passed in when the method is used if the object is created in this class itself, it will not allow itself to be used other than as a null object and then will lead to error: NullPointerException, cannot call "{whatever}" on null object reference. **/

        Pose2d STTcurrentPose = new Pose2d(0,0,0);

        drive.setPoseEstimate(STTcurrentPose);
        drive.setDrivePower(new Pose2d(0, 0, 1));

        double currentVoltage = batteryVoltageSensor.getVoltage();
        double base = OPTIMAL_BATTERY_VOLTAGE / currentVoltage;
        double exponent = (((Math.log(BASE_SCALE_FACTOR) / Math.log(OPTIMAL_BATTERY_VOLTAGE / currentVoltage)) + (Math.log(SCALE_FACTOR2) / Math.log(OPTIMAL_BATTERY_VOLTAGE2 / currentVoltage))) / 2);

        telemetry.addData("exponent: ", exponent);
        telemetry.update();

        timer.reset();
        while(timer.milliseconds() <= (FAST_TURN_WAIT_TIME * Math.pow(base, exponent))) {
            telemetry.addLine("Turning");
            telemetry.update();
        }
        telemetry.clearAll();
        drive.setDrivePower(new Pose2d(0, 0, -1));  /**Braking system - limiting deceleration**/
        sleep(1);                                      /**Braking system - limiting deceleration**/
        drive.setDrivePower(new Pose2d(0, 0, 0));   /**Braking system - limiting deceleration**/

        drive.updatePoseEstimate();
        if (drive.getPoseEstimate() != STTcurrentPose) { //spline adjustment
            Trajectory fixPose = drive.trajectoryBuilder(drive.getPoseEstimate()).splineToConstantHeading(new Vector2d(STTcurrentPose.getX(), STTcurrentPose.getY()), drive.getPoseEstimate().getHeading()).build();
            drive.followTrajectoryAsync(fixPose);
        }
        telemetry.addLine("Fast Turn Completed!");
        telemetry.update();
        drive.waitForIdle();
        telemetry.addData("base", base);
        telemetry.addData("exponent", exponent);
        telemetry.addData("base", Math.pow(base, exponent));
        telemetry.update();
        telemetry.addLine("Drive Idle.");
        telemetry.update();
    }


    public void runOpMode() {/* --EMPTY-- */}
}