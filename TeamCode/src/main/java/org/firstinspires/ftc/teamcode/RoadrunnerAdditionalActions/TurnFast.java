package org.firstinspires.ftc.teamcode.RoadrunnerAdditionalActions;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Config //Can edit FAST_TURN_WAIT_TIME, OPTIMAL_BATTERY_VOLTAGEs, and TRAINED_EXP_MULTIPLIERs in FTC Dashboard
@Autonomous (name = "TurnFast")
public class TurnFast extends LinearOpMode {

    // TODO: Tune MAX_VEL ~ DONE
    // TODO: Tune MAX_ACCEL ~ DONE
    // TODO: Tune/Check MAX_ANG_ACCEL ~ DONE - usually already perfect

    public static double FAST_TURN_WAIT_TIME = 630;
    public static double TRAINED_EXP_MULTIPLIER1 = 1.0;
    public static double TRAINED_EXP_MULTIPLIER2 = 0.995875;
    public static double TRAINED_EXP_MULTIPLIER3 = 0.9475;
    public static double TRAINED_EXP_MULTIPLIER4 = 0.88;

    public static double OPTIMAL_BATTERY_VOLTAGE1 = 12.7321;
    public static double OPTIMAL_BATTERY_VOLTAGE2 = 12.89;
    public static double OPTIMAL_BATTERY_VOLTAGE3 = 12.95;
    public static double OPTIMAL_BATTERY_VOLTAGE4 = 13.632;

    private ElapsedTime timer = new ElapsedTime();

    public void turnFast(double fastTurnAmount, SampleMecanumDrive drive, VoltageSensor batteryVoltageSensor, Telemetry telemetry) {

        if (drive == null || batteryVoltageSensor == null || fastTurnAmount == 0) return;

        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            /** drive = new SampleMecanumDrive(hardwareMap); is not applied because that parameter is passed in when the method is used if the object is created in this class itself, it will not allow itself to be used other than as a null object and then will lead to error: NullPointerException, cannot call "{whatever}" on null object reference. **/

        Pose2d STTcurrentPose = new Pose2d(0,0,0);

        drive.setPoseEstimate(STTcurrentPose);
        drive.setDrivePower(new Pose2d(0, 0, 1));
        //sleep(FAST_TURN_WAIT_TIME * ((long) OPTIMAL_BATTERY_VOLTAGE / (long) batteryVoltageSensor.getVoltage())); //wait time * (primal voltage / actual voltage)

        double base = ((OPTIMAL_BATTERY_VOLTAGE1 + OPTIMAL_BATTERY_VOLTAGE2 + OPTIMAL_BATTERY_VOLTAGE3 + OPTIMAL_BATTERY_VOLTAGE4) / 4) / batteryVoltageSensor.getVoltage();
        double exponent = (((Math.log(TRAINED_EXP_MULTIPLIER1) / Math.log(OPTIMAL_BATTERY_VOLTAGE1)) + (Math.log(TRAINED_EXP_MULTIPLIER2) / Math.log(OPTIMAL_BATTERY_VOLTAGE3)) + (Math.log(TRAINED_EXP_MULTIPLIER3) / Math.log(OPTIMAL_BATTERY_VOLTAGE3)) + (Math.log(TRAINED_EXP_MULTIPLIER4) / Math.log(OPTIMAL_BATTERY_VOLTAGE4))) / 4);

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
        telemetry.addLine("Drive Idle.");
        telemetry.update();
    }


    public void runOpMode() {/* --EMPTY-- */}
}