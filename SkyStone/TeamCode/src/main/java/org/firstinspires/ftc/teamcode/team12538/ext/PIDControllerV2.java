package org.firstinspires.ftc.teamcode.team12538.ext;

import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

public class PIDControllerV2 {
    private int target;
    private int sensorInput;
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double error = 0;
    private double previousError = 0;
    private double integral = 0;
    private double integralThreshold = 10;
    private double derivative = 0;
    private long delay = 0;

    public PIDControllerV2(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double performPID(int sensorInput) {
        this.sensorInput = sensorInput;
        return calculate();
    }

    public int getTarget() {
        return target;
    }

    public void setTarget(int target) {
        this.target = target;
    }

    public long getDelay() {
        return delay;
    }

    public void setDelay(long delay) {
        this.delay = delay;
    }

    public void reset()
    {
        error = 0;
        previousError = 0;
        integral = 0;
        derivative = 0;
    }

    private double calculate() {
        // Save the previous error for the derivative
        this.previousError = error;

        // Calculate a new error by finding the difference between the current position
        // and the desired position.
        this.error = target - sensorInput;

        // Begin summing the errors into the integral term if the error is below a threshold,
        // and reset it if not. This is to prevent the integral from growing too large.
        if(Math.abs(error) < integralThreshold) {
            integral += error;
        } else {
            integral = 0;
        }

        // Calculate the derivative by finding the change between the current error and
        // last update's error
        derivative = error - previousError;

        if(delay > 0) {
            ThreadUtils.sleep(delay);
        }

        // Combine all the parts of the PID function into the PID algorithm and return the value.
        return (int) ((kP * error) + (kI * integral) + (kD * derivative));
    }
}
