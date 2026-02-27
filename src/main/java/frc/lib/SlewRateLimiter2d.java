// https://github.com/amsam0/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/filter/SlewRateLimiter.java
// https://github.com/wpilibsuite/allwpilib/pull/8581

// Copyright (c) 2009-2026 FIRST and other WPILib contributors
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of FIRST, WPILib, nor the names of other WPILib
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY FIRST AND OTHER WPILIB CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY NONINFRINGEMENT AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL FIRST OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 * <p>
 * Now with support for TWO DIMENSIONS!!!! Ensures that the norm/magnitude of change does not
 * exceed given rate limit.
 */
public class SlewRateLimiter2d {
    private double m_positiveRateLimit;
    private double m_negativeRateLimit;
    private Translation2d m_prevVal;
    private double m_prevTime;

    /**
     * Creates a new SlewRateLimiter with the given positive and negative rate limits and initial
     * value.
     *
     * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
     *                          second. This is expected to be positive.
     * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
     *                          second. This is expected to be negative.
     * @param initialValue      The initial value of the input.
     */
    public SlewRateLimiter2d(double positiveRateLimit, double negativeRateLimit, Translation2d initialValue) {
        m_positiveRateLimit = positiveRateLimit;
        m_negativeRateLimit = negativeRateLimit;
        m_prevVal = initialValue;
        m_prevTime = MathSharedStore.getTimestamp();
    }

    /**
     * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
     * -rateLimit.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public SlewRateLimiter2d(double rateLimit) {
        this(rateLimit, -rateLimit, new Translation2d());
    }

    public SlewRateLimiter2d(double rateLimit, ChassisSpeeds initialValue) {
        this(rateLimit, -rateLimit, new Translation2d(initialValue.vxMetersPerSecond, initialValue.vyMetersPerSecond));
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     *
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public Translation2d calculate(Translation2d input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - m_prevTime;
        Translation2d wantedChange = input.minus(m_prevVal);
        double norm =
                MathUtil.clamp(
                        wantedChange.getNorm(),
                        m_negativeRateLimit * elapsedTime,
                        m_positiveRateLimit * elapsedTime
                );
        Translation2d realChange = norm == 0.0 // if 0, then Rotation2d will throw a warning
                ? new Translation2d()
                : new Translation2d(norm, wantedChange.getAngle());
        m_prevVal = m_prevVal.plus(realChange);

        m_prevTime = currentTime;
        return m_prevVal;
    }

    /**
     * Returns the value last calculated by the SlewRateLimiter.
     *
     * @return The last value.
     */
    public Translation2d lastValue() {
        return m_prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(Translation2d value) {
        m_prevVal = value;
        m_prevTime = MathSharedStore.getTimestamp();
    }

    public void reset(ChassisSpeeds value) {
        reset(new Translation2d(value.vxMetersPerSecond, value.vyMetersPerSecond));
    }

    /**
     * Sets the rate-of-change limit to the given positive and negative rate limits.
     *
     * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
     *                          second. This is expected to be positive.
     * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
     *                          second. This is expected to be negative.
     */
    public void setLimit(double positiveRateLimit, double negativeRateLimit) {
        m_positiveRateLimit = positiveRateLimit;
        m_negativeRateLimit = negativeRateLimit;
    }

    /**
     * Sets the rate-of-change limit to the given positive rate limit and negative rate limit of
     * -rateLimit.
     *
     * @param rateLimit The rate-of-change limit in both directions, in units per second. This is
     *                  expected to be positive.
     */
    public void setLimit(double rateLimit) {
        m_positiveRateLimit = rateLimit;
        m_negativeRateLimit = -rateLimit;
    }
}
