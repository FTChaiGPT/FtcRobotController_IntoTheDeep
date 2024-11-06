package org.firstinspires.ftc.teamcode.RoadrunnerAdditionalActions;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Objects;

@Config //Can edit FAST_TURN_WAIT_TIME and OPTIMAL_BATTERY_VOLTAGE in FTC Dashboard
@Autonomous(name = "TurnFast")
public class TurnFast extends LinearOpMode {

    // TODO: Tune MAX_VEL ~ DONE
    // TODO: Tune MAX_ACCEL ~ DONE
    // TODO: Tune/Check MAX_ANG_ACCEL ~ probably already perfect

    public static long FAST_TURN_WAIT_TIME = 630;  //TODO: Needs to be tuned
    public static double OPTIMAL_BATTERY_VOLTAGE = 12.973;

    public void turnFast(long fastTurnAmount, SampleMecanumDrive drive, VoltageSensor batteryVoltageSensor, Telemetry telemetry) {

        if (drive == null || batteryVoltageSensor == null || fastTurnAmount == 0) return;

        drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            /** drive = new SampleMecanumDrive(hardwareMap); is not applies because that parameter is passed in when the method is used if the object is created in this class itself, it will not allow itself to be used other than as a null object and then will lead to error: NullPointerException, cannot call "{whatever}" on null object reference. **/

        Pose2d STTcurrentPose = new Pose2d(0,0,0);

        drive.setPoseEstimate(STTcurrentPose);
        drive.setDrivePower(new Pose2d(0, 0, 1));
        sleep(FAST_TURN_WAIT_TIME * ((long) OPTIMAL_BATTERY_VOLTAGE / (long) batteryVoltageSensor.getVoltage())); /**Get primal voltage**/ // * (primal voltage / actual voltage)
        drive.setDrivePower(new Pose2d(0, 0, -1));  /**Braking system - limiting deceleration**/
        sleep(1);                                      /**Braking system - limiting deceleration**/
        drive.setDrivePower(new Pose2d(0, 0, 0));   /**Braking system - limiting deceleration**/

        drive.updatePoseEstimate();
        if (drive.getPoseEstimate() != STTcurrentPose) { //spline adjustment
            Trajectory fixPose = drive.trajectoryBuilder(drive.getPoseEstimate()).splineToConstantHeading(new Vector2d(STTcurrentPose.getX(), STTcurrentPose.getY()), STTcurrentPose.getHeading()).build();
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