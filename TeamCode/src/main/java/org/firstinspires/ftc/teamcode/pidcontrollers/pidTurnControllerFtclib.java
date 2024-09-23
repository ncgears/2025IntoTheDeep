package org.firstinspires.ftc.teamcode.pidcontrollers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Constant;
import org.firstinspires.ftc.teamcode.Constants;

public class pidTurnControllerFtclib {
    private double targetAngle;
    private double kP, kI, kD, kF, kIZone;
    private double accumulatedError = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError, lastTime = 0.0;
    private OpMode myOpMode = null;

    public pidTurnControllerFtclib(OpMode opmode, double target, double p, double i, double d, double f, double iZone) {
        myOpMode = opmode;
        targetAngle = target;
        kP = p;
        kI = i;
        kIZone = iZone;
        kD = d;
        kF = f;
    }
    public double update(double currentAngle) {
        //P - Proportional - This determines the error that we will multiply by our constant to set power
        double error = targetAngle - currentAngle;
        error %= 360;
        error += 360;
        error %= 360;
        if (error > 180) error -= 360;

        //I - Integral - This accumulates the error over time to correct for not getting to the set point
        if(Math.abs(error) < kIZone) {
            accumulatedError += error;
            //if we reach the threshold, reset accumulated error to stop adding it
            if (atTarget(currentAngle)) {
                accumulatedError = 0;
            }
            accumulatedError = Math.abs(accumulatedError) * Math.signum(error);
        }

        //D - Derivative - This slows down the robot when its moving too rapidly
        double slope = 0.0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastTime = timer.milliseconds();
        lastError = error;
//      myOpMode.telemetry.addData("slope","%.2f", slope);
        //Motor Power calculation
        double motorPower = kF * Math.signum(error) + (1.0 - kF) * Math.tanh(
                (kP * error) + (kI * accumulatedError) + (kD * slope)
        );

        //Limit output to max value
        motorPower = Math.min(Math.abs(motorPower),Constants.Drivetrain.turnController.limits.maxOutput) * Math.signum(motorPower);

        return motorPower;
    }
    public double getTarget() {
        return targetAngle;
    }

    public void setTarget(double degrees) {
        targetAngle = degrees;
    }

    public double getLastError() {
        return lastError;
    }

    public boolean atTarget(double currentAngle) {
        double error = targetAngle - currentAngle;
        error %= 360;
        error += 360;
        error %= 360;
        if (error > 180) error -= 360;
        return (Math.abs(error) < Constants.Drivetrain.turnController.targetThreshold);
    }
}
