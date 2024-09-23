package org.firstinspires.ftc.teamcode.pidcontrollers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

public class pidDriveControllerFtclib {
    private double targetTicks;
    private double kP, kI, kD, kF, kIZone;
    private double accumulatedError = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError, lastTime = 0.0;
    private OpMode myOpMode = null;

    public pidDriveControllerFtclib(OpMode opmode, double target, double p, double i, double d, double f, double iZone) {
        myOpMode = opmode;
        targetTicks = target;
        kP = p;
        kI = i;
        kIZone = iZone;
        kD = d;
        kF = f;
    }
    public double update(double currentTicks) {
        //P - Proportional - This determines the error that we will multiply by our constant to set power
        double error = targetTicks - currentTicks;
//        myOpMode.telemetry.addData("drive error", "%.1f", error);
//        RobotLog.d(String.format("drive error = %.2f", error));
//        myOpMode.telemetry.update();
        //I - Integral - This accumulates the error over time to correct for not getting to the set point
        if(Math.abs(error) < kIZone) {
            accumulatedError += error;
            //if we reach the threshold, reset accumulated error to stop adding it
            if (atTarget(error)) {
                accumulatedError = 0;
            }
            accumulatedError = Math.abs(accumulatedError) * Math.signum(error);
        }

        //D - Deriviative - This slows down the robot when its moving too rapidly
        double slope = 0.0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastTime = timer.milliseconds();
        lastError = error;

        //Motor Power calculation
        double motorPower = kF * Math.signum(error) + (1.0 - kF) * Math.tanh(
                (kP * error) + (kI * accumulatedError) + (kD * slope)
        );

        //Limit output to max value
        motorPower = Math.min(Math.abs(motorPower),Constants.Drivetrain.driveController.limits.maxOutput) * Math.signum(motorPower);

        return motorPower;
    }

    public double getTarget() {
        return targetTicks;
    }

    public void setTargetInches(double inches) { setTarget(inches * Constants.Drivetrain.driveController.ticksPerInch); }

    public void setTarget(double ticks) {
        targetTicks = ticks;
    }

    public double getLastError() {
        return lastError;
    }

    public boolean atTarget(double error) {
        return (Math.abs(error) < Constants.Drivetrain.driveController.targetThresholdTicks);
    }
    public boolean atTarget() {
        return (Math.abs(lastError) < Constants.Drivetrain.driveController.targetThresholdTicks);
    }
}
