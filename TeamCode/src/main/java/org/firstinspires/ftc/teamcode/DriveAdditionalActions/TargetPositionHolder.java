package org.firstinspires.ftc.teamcode.DriveAdditionalActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@Config
@Autonomous (name = "TargetPositionHolder")
public class TargetPositionHolder extends LinearOpMode {
    public void runOpMode() {/*--EMPTY--*/}

    private volatile boolean isMotorHolding = false;
    private ExecutorService executor = Executors.newSingleThreadExecutor();
    private double power;

    public static double TRAINED_TICKS_PER_REV = 537.7;

    private volatile double TICKS_PER_REV = 537.7;
    public volatile double gearRatio = 1; /** GEAR_RATIO output (motor) speed / input (motor) speed **/
    public volatile double holdPower = 0; /** power set once motor has reached its TargetPosition **/
    public volatile double marginOfError = 5 ;
    public volatile double powerMultiplier = 1;
    public volatile double tuningPowerMultiplier = 1; /**power multiplier that adjustments are made at**/

    private volatile Map<DcMotor, Future<?>> motorTasks = new HashMap<>(); /** HashMap to create instances of classes **/ //holds all Futures of the ExecutorService
    /** Future is used to handle specific parts of the ExecutorService instead of only handling all or none **/


    public void holdDcMotor(@NonNull DcMotor motor, @NonNull double holdPosition, @NonNull VoltageSensor batteryVoltageSensor, Object... varargs) {

        double[] varargsData = processVarargs(varargs);
        gearRatio = varargsData[0];
        TICKS_PER_REV = varargsData[1];
        holdPower = varargsData[2];
        marginOfError = varargsData[3];
        powerMultiplier = varargsData[4];
        tuningPowerMultiplier = varargsData[5];

        double MAX_POWER = ((1 / gearRatio) == 1) ? 1 : Math.min(1 / gearRatio, 1);                                                                                                 /** ternary operator **/
        double LOWER_MAX_POWER = ((MAX_POWER / 1.3) * powerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : ((MAX_POWER / 1.3) * powerMultiplier);  /** ternary operator **/
        double HALF_POWER = ((MAX_POWER / 5.5) * powerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : ((MAX_POWER / 2) * powerMultiplier);         /** ternary operator **/
        double MOD_POW = ((MAX_POWER / 7.5) * powerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : ((MAX_POWER / 3.5) * powerMultiplier);          /** ternary operator **/
        double LOW_POW = ((MAX_POWER / 9) * powerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : ((MAX_POWER / 8) * powerMultiplier);              /** ternary operator **/
        double MIN_POW = ((MAX_POWER / 10) * powerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : ((MAX_POWER / 9.5) * powerMultiplier);           /** ternary operator **/
        double LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POW = ((MAX_POWER / 5) * powerMultiplier) > 1 ? 1 : ((MAX_POWER / 1.3) * powerMultiplier);                                       /** ternary operator **/

        /** tuningPowerMultiplier is used after powerMultiplier is used as well **/
        double MOD_POWER = (MOD_POW * tuningPowerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : (MOD_POW * tuningPowerMultiplier);                    /** ternary operator **/
        double LOW_POWER = (LOW_POW * tuningPowerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : (LOW_POW * tuningPowerMultiplier);                    /** ternary operator **/
        double MIN_POWER = (MIN_POW * tuningPowerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : (MIN_POW * tuningPowerMultiplier);                /** ternary operator **/
        double LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POWER = (LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POW * tuningPowerMultiplier) > 1 ? 1 : (LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POW * tuningPowerMultiplier);/** ternary operator **/


        isMotorHolding = true;

        Future<?> motorTask = executor.submit(() -> {

            motor.setTargetPosition((int) holdPosition);

            try {
                while (isMotorHolding) {

                    power = setAppropriatePower(MAX_POWER, LOWER_MAX_POWER, HALF_POWER, MOD_POWER, LOW_POWER, MIN_POWER, LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POWER, batteryVoltageSensor, motor);

                    if (Math.floor(Math.abs(motor.getCurrentPosition())) > (Math.abs(motor.getTargetPosition()) - marginOfError) && Math.floor(Math.abs(motor.getCurrentPosition())) < (Math.abs(motor.getTargetPosition()) + marginOfError)) {
                        motor.setPower(holdPower);
                    } else {
                        motor.setPower(power);
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                }
            } catch (Exception e) {
                Thread.currentThread().interrupt();
            } finally { clearFutureOfDcMotor(motor); }

        });
        if (motorTasks.containsKey(motor) == false) {
            motorTasks.put(motor, motorTask); //motor task is added to MotorTasks
        }

    }


    public double[] processVarargs(Object... varargs) {

        double ticksPerRev = 537.7;
        double gearRatio = 1.0;
        double holdPower = 0.0;
        double marginOfError = 5.0;
        double powerMultiplier = 1.0;
        double tuningPowerMultiplier = 1.0;

        for (int i = 0; i < varargs.length; i += 2) {
            if (varargs[i] instanceof String && varargs[i + 1] instanceof Number) {
                String stringKey = (String) varargs[i];
                double value = ((Number) varargs[i + 1]).doubleValue();
                /** Number is data type of Object number, here it is Double,
                 * Double is a wrapper of the primitive: double, and needs to be unbox Double to double avoids ClassCastException **/
                switch (stringKey) {
                    case "GEAR_RATIO":
                        gearRatio = value;
                        break;
                    case "TICKS_PER_REV":
                        ticksPerRev = value;
                        break;
                    case "HOLD_POWER":
                        holdPower = value;
                        break;
                    case "ALLOWABLE_MARGIN_OF_ERROR":
                        marginOfError =  value;
                        break;
                    case "POWER_MULTIPLIER":
                        powerMultiplier = value;
                        break;
                    case "TUNING_POWER_MULTIPLIER":
                        tuningPowerMultiplier = value;
                    default:
                        //does nothing for unknown keys.
                        break;
                }
            }
        }
        return new double[] {gearRatio, ticksPerRev, holdPower, marginOfError, powerMultiplier, tuningPowerMultiplier};
    }

    public void clearFutureOfDcMotor(DcMotor motor) {
        if (motorTasks.containsKey(motor)) motorTasks.remove(motor);
    }

    public void clearTasks() {
        motorTasks.clear();
    }

    public void killExecutor(DcMotor motor, VoltageSensor batteryVoltageSensor) {
        stopHoldingDcMotor(motor);
        clearTasks();
        executor.shutdown();
    }

    public void stopHoldingDcMotor(DcMotor motor) {
        Future<?> motorTask = motorTasks.get(motor);
        if (motorTask != null) {
            motorTask.cancel(true);
        }
        isMotorHolding = false;
        motor.setPower(0.0);
    }

    public double setAppropriatePower(double MAX_POWER,
                                      double LOWER_MAX_POWER,
                                      double HALF_POWER,
                                      double MOD_POWER,
            /* gets motor powers */      double LOW_POWER,
                                      double MIN_POWER,
                                      double LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POWER,
                                      VoltageSensor batteryVoltageSensor,
                                      DcMotor motor)
        /* start of loop >>> */ {
        double powerAmount;
        double positionDifference = Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
        double currentBatteryVoltage = batteryVoltageSensor.getVoltage();
        //evaluates motor powers
        /** Tuneable values **/
        if (positionDifference <= TICKS_PER_REV * (35 / TRAINED_TICKS_PER_REV)) {
            if (currentBatteryVoltage <= 11.75) {
                powerAmount = LOW_POWER;
            }
            else if (currentBatteryVoltage <= 12) {
                powerAmount = LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POWER;
            }
            else powerAmount = MIN_POWER;
        } else if (positionDifference <= TICKS_PER_REV * (500 / TRAINED_TICKS_PER_REV)) {
            if (currentBatteryVoltage <= 11.75) {
                powerAmount = LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POWER;
            }
            else powerAmount = LOW_POWER;
        } else if (positionDifference <= TICKS_PER_REV * (900 / TRAINED_TICKS_PER_REV)) {
            powerAmount = MOD_POWER;
        } else if (positionDifference <= TICKS_PER_REV * (1200 / TRAINED_TICKS_PER_REV)) {
            powerAmount = HALF_POWER;
        } else if (positionDifference <= TICKS_PER_REV * (2500 / TRAINED_TICKS_PER_REV)) {
            powerAmount = LOWER_MAX_POWER;
        }
        else powerAmount = MAX_POWER;

        return powerAmount;
    }
}