package org.firstinspires.ftc.teamcode.utils;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage;
 */

public class SlewRateLimiter {
    private  double m_positiveRateLimit=0;
    private double m_negativeRateLimit=0;
    private double m_prevVal;
    private double m_prevTime;

    ElapsedTime eltime=new ElapsedTime();

    /**
     * Creates a new SlewRateLimiter with the given positive and negative rate limits and initial
     * value.
     *
     * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
     *     second. This is expected to be positive.
     * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
     *     second. This is expected to be negative.
     * @param initialValue The initial value of the input.
     */

    public SlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        m_positiveRateLimit = positiveRateLimit;
        m_negativeRateLimit = negativeRateLimit;
        m_prevVal = initialValue;
        m_prevTime = eltime.seconds();
    }


    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input) {
        double currentTime = eltime.seconds();
        double elapsedTime = currentTime - m_prevTime;

        m_prevVal +=clamp(
                (input - m_prevVal),
                (m_negativeRateLimit * elapsedTime),
                (m_positiveRateLimit * elapsedTime));

        m_prevTime = currentTime;
        return m_prevVal;
    }

    public double clamp(double value, double low, double high){
      return  Math.max(low, Math.min(value, high));
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        m_prevVal = value;
        m_prevTime=eltime.seconds();
    }
}
