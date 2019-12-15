package org.firstinspires.ftc.teamcode.team12538.ext;

// PID controller courtesy of Peter Tischler, with modifications.

public class PIDControllerV1
{
    private double m_P;                     // factor for "proportional" control
    private double m_I;                     // factor for "integral" control
    private double m_D;                     // factor for "derivative" control
    private double m_input;                 // sensor input for pid controller
    private double m_maximumOutput = 1.0;	// |maximum output|
    private double m_minimumOutput = -1.0;	// |minimum output|
    private double m_maximumInput = 0.0;	// maximum input - limit setpoint to this
    private double m_minimumInput = 0.0;	// minimum input - limit setpoint to this
    private boolean m_continuous = false;	// do the endpoints wrap around? eg. Absolute encoder
    private boolean m_enabled = true;       // is the pid controller enabled
    private double m_prevError = 0.0;       // the prior sensor input (used to compute velocity)
    private double m_totalError = 0.0;      // the sum of the errors for use in the integral calc
    private double m_tolerance = 0.05;      // the percentage error that is considered on target
    private double m_target = 0.0;
    private double m_error = 0.0;
    private double m_result = 0.0;

    /**
     * Allocate a PID object with the given constants for P, I, D
     * @param Kp the proportional coefficient
     * @param Ki the integral coefficient
     * @param Kd the derivative coefficient
     */
    public PIDControllerV1(double Kp, double Ki, double Kd)
    {
        m_P = Kp;
        m_I = Ki;
        m_D = Kd;
    }

    /**
     * Read the input, calculate the output accordingly, and write to the output.
     * This should only be called by the PIDTask
     * and is created during initialization.
     */
    private void calculate()
    {
        int     sign = 1;

        // If enabled then proceed into controller calculations
        if (m_enabled)
        {
            // Calculate the error signal
            m_error = m_target - m_input;

            // If continuous is set to true allow wrap around
            if (m_continuous)
            {
                if (Math.abs(m_error) > (m_maximumInput - m_minimumInput) / 2)
                {
                    if (m_error > 0)
                        m_error = m_error - m_maximumInput + m_minimumInput;
                    else
                        m_error = m_error + m_maximumInput - m_minimumInput;
                }
            }

            // Integrate the errors as long as the upcoming integrator does
            // not exceed the minimum and maximum output thresholds.

            if ((Math.abs(m_totalError + m_error) * m_I < m_maximumOutput) &&
                    (Math.abs(m_totalError + m_error) * m_I > m_minimumOutput))
                m_totalError += m_error;

            // Perform the primary PID calculation
            m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError);

            // Set the current error to the previous error for the next cycle.
            m_prevError = m_error;

            if (m_result < 0) sign = -1;    // Record sign of result.

            // Make sure the final result is within bounds. If we constrain the result, we make
            // sure the sign of the constrained result matches the original result sign.
            if (Math.abs(m_result) > m_maximumOutput)
                m_result = m_maximumOutput * sign;
            else if (Math.abs(m_result) < m_minimumOutput)
                m_result = m_minimumOutput * sign;
        }
    }

    /**
     * Set the PID Controller gain parameters.
     * Set the proportional, integral, and differential coefficients.
     * @param p Proportional coefficient
     * @param i Integral coefficient
     * @param d Differential coefficient
     */
    public void setPID(double p, double i, double d)
    {
        m_P = p;
        m_I = i;
        m_D = d;
    }

    /**
     * Get the Proportional coefficient
     * @return proportional coefficient
     */
    public double getP()
    {
        return m_P;
    }

    /**
     * Get the Integral coefficient
     * @return integral coefficient
     */
    public double getI()
    {
        return m_I;
    }

    /**
     * Get the Differential coefficient
     * @return differential coefficient
     */
    public double getD()
    {
        return m_D;
    }

    /**
     * Return the current PID result for the last input set with setInput().
     * This is always centered on zero and constrained the the max and min outs
     * @return the latest calculated output
     */
    public double performPID()
    {
        calculate();
        return m_result;
    }

    /**
     * Return the current PID result for the specified input.
     * @param input The input value to be used to calculate the PID result.
     * This is always centered on zero and constrained the the max and min outs
     * @return the latest calculated output
     */
    public double performPID(double input)
    {
        setInput(input);
        return performPID();
    }

    /**
     *  Set the PID controller to consider the input to be continuous,
     *  Rather then using the max and min in as constraints, it considers them to
     *  be the same point and automatically calculates the shortest route to
     *  the setpoint.
     * @param continuous Set to true turns on continuous, false turns off continuous
     */
    public void setContinuous(boolean continuous)
    {
        m_continuous = continuous;
    }

    /**
     *  Set the PID controller to consider the input to be continuous,
     *  Rather then using the max and min in as constraints, it considers them to
     *  be the same point and automatically calculates the shortest route to
     *  the setpoint.
     */
    public void setContinuous()
    {
        this.setContinuous(true);
    }

    /**
     * Sets the maximum and minimum values expected from the input.
     *
     * @param minimumInput the minimum value expected from the input, always positive
     * @param maximumInput the maximum value expected from the output, always positive
     */
    public void setInputRange(double minimumInput, double maximumInput)
    {
        m_minimumInput = Math.abs(minimumInput);
        m_maximumInput = Math.abs(maximumInput);
        setTarget(m_target);
    }

    /**
     * Sets the minimum and maximum values to write.
     *
     * @param minimumOutput the minimum value to write to the output, always positive
     * @param maximumOutput the maximum value to write to the output, always positive
     */
    public void setOutputRange(double minimumOutput, double maximumOutput)
    {
        m_minimumOutput = Math.abs(minimumOutput);
        m_maximumOutput = Math.abs(maximumOutput);
    }

    /**
     * Set the setpoint for the PIDControllerV1
     * @param target the desired setpoint
     */
    public void setTarget(double target) {
        int     sign = 1;
        if (m_maximumInput > m_minimumInput) {
            if (target < 0) {
                sign = -1;
            }

            if (Math.abs(target) > m_maximumInput) {
                m_target = m_maximumInput * sign;
            } else if (Math.abs(target) < m_minimumInput) {
                m_target = m_minimumInput * sign;
            } else {
                m_target = target;
            }
        } else {
            m_target = target;
        }
    }

    /**
     * Returns the current setpoint of the PIDControllerV1
     * @return the current setpoint
     */
    public double getTarget()
    {
        return m_target;
    }

    /**
     * Retruns the current difference of the input from the setpoint
     * @return the current error
     */
    public synchronized double getError()
    {
        return m_error;
    }

    /**
     * Set the percentage error which is considered tolerable for use with
     * OnTarget. (Input of 15.0 = 15 percent)
     * @param percent error which is tolerable
     */
    public void setTolerance(double percent)
    {
        m_tolerance = percent;
    }

    /**
     * Return true if the error is within the percentage of the total input range,
     * determined by setTolerance. This assumes that the maximum and minimum input
     * were set using setInputRange.
     * @return true if the error is less than the tolerance
     */
    public boolean onTarget()
    {
        return (Math.abs(m_error) < Math.abs(m_tolerance / 100.0 * (m_maximumInput - m_minimumInput)));
    }

    /**
     * Begin running the PIDControllerV1
     */
    public void enable()
    {
        m_enabled = true;
    }

    /**
     * Stop running the PIDControllerV1.
     */
    public void disable()
    {
        m_enabled = false;
    }

    /**
     * Reset the previous error,, the integral term, and disable the controller.
     */
    public void reset()
    {
        m_prevError = 0;
        m_totalError = 0;
        m_result = 0;
    }

    /**
     * Set the input value to be used by the next call to performPID().
     * @param input Input value to the PID calculation.
     */
    public void setInput(double input)
    {
        int     sign = 1;

        if (m_maximumInput > m_minimumInput)
        {
            if (input < 0) sign = -1;

            if (Math.abs(input) > m_maximumInput)
                m_input = m_maximumInput * sign;
            else if (Math.abs(input) < m_minimumInput)
                m_input = m_minimumInput * sign;
            else
                m_input = input;
        }
        else
            m_input = input;
    }
}
