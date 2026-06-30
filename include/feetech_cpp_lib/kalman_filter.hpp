/// \file kalman_filter.hpp
/// \brief Per-joint Kalman filter fusing position and velocity measurements.
///
/// \details The joint is modelled with a constant-velocity motion model and a
///          2-D state x = [position, velocity]. Both the position and the
///          velocity are measured directly (the servo reports both), so the
///          measurement model is the identity and the system is fully linear.
///          For a linear model the Extended Kalman Filter reduces exactly to
///          the standard (linear) Kalman Filter, so that is what is implemented
///          here -- it is the optimal estimator for this problem and avoids the
///          needless Jacobian machinery of a general EKF.
///
///          Process model (acceleration treated as zero-mean white noise):
///              p_{k+1} = p_k + v_k * dt
///              v_{k+1} = v_k + w,   w ~ N(0, sigma_a^2 * dt^2)
///
///          Measurement model:
///              z = [p_meas; v_meas] = I * x + noise
#ifndef FEETECH_KALMAN_FILTER_HPP
#define FEETECH_KALMAN_FILTER_HPP

struct JointKalman
{
    // State estimate: x0 = position [rad], x1 = velocity [rad/s].
    double x0 = 0.0;
    double x1 = 0.0;

    // State covariance (symmetric 2x2).
    double P00 = 1.0, P01 = 0.0;
    double P10 = 0.0, P11 = 1.0;

    bool initialized = false;

    /// \brief Fuse a new position/velocity measurement.
    /// \param[in] z_pos    Measured position [rad] (continuous / unwrapped).
    /// \param[in] z_vel    Measured velocity [rad/s].
    /// \param[in] dt       Time since the previous update [s].
    /// \param[in] sigma_a  Process noise: std. dev. of acceleration [rad/s^2].
    /// \param[in] r_pos    Position measurement noise std. dev. [rad].
    /// \param[in] r_vel    Velocity measurement noise std. dev. [rad/s].
    void update(double z_pos, double z_vel, double dt,
                double sigma_a, double r_pos, double r_vel)
    {
        if (!initialized || dt <= 0.0)
        {
            // (Re)initialize directly from the measurement.
            x0 = z_pos;
            x1 = z_vel;
            P00 = r_pos * r_pos; P01 = 0.0;
            P10 = 0.0;           P11 = r_vel * r_vel;
            initialized = true;
            return;
        }

        /* ---- Predict ---- */
        // x = F x, with F = [[1, dt], [0, 1]]
        x0 = x0 + x1 * dt;
        // x1 unchanged by the prediction.

        // P = F P F^T + Q
        // FP = F * P
        const double FP00 = P00 + dt * P10;
        const double FP01 = P01 + dt * P11;
        const double FP10 = P10;
        const double FP11 = P11;
        // P = FP * F^T, with F^T = [[1, 0], [dt, 1]]
        double nP00 = FP00 + dt * FP01;
        double nP01 = FP01;
        double nP10 = FP10 + dt * FP11;
        double nP11 = FP11;

        // Process noise Q from a discretized white-noise-acceleration model.
        const double q = sigma_a * sigma_a;
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt3 * dt;
        nP00 += q * dt4 / 4.0;
        nP01 += q * dt3 / 2.0;
        nP10 += q * dt3 / 2.0;
        nP11 += q * dt2;

        P00 = nP00; P01 = nP01; P10 = nP10; P11 = nP11;

        /* ---- Update (measurement model H = I) ---- */
        // Innovation covariance S = P + R
        const double S00 = P00 + r_pos * r_pos;
        const double S01 = P01;
        const double S10 = P10;
        const double S11 = P11 + r_vel * r_vel;

        double det = S00 * S11 - S01 * S10;
        if (det == 0.0)
            return;  // Degenerate; skip the correction this step.
        const double invDet = 1.0 / det;
        const double Si00 =  S11 * invDet;
        const double Si01 = -S01 * invDet;
        const double Si10 = -S10 * invDet;
        const double Si11 =  S00 * invDet;

        // Kalman gain K = P * S^{-1}
        const double K00 = P00 * Si00 + P01 * Si10;
        const double K01 = P00 * Si01 + P01 * Si11;
        const double K10 = P10 * Si00 + P11 * Si10;
        const double K11 = P10 * Si01 + P11 * Si11;

        // Innovation y = z - x
        const double y0 = z_pos - x0;
        const double y1 = z_vel - x1;

        // State correction x = x + K y
        x0 += K00 * y0 + K01 * y1;
        x1 += K10 * y0 + K11 * y1;

        // Covariance update P = (I - K) P
        const double IK00 = 1.0 - K00, IK01 = -K01;
        const double IK10 = -K10,      IK11 = 1.0 - K11;
        const double uP00 = IK00 * P00 + IK01 * P10;
        const double uP01 = IK00 * P01 + IK01 * P11;
        const double uP10 = IK10 * P00 + IK11 * P10;
        const double uP11 = IK10 * P01 + IK11 * P11;
        P00 = uP00; P01 = uP01; P10 = uP10; P11 = uP11;
    }

    double position() const { return x0; }
    double velocity() const { return x1; }
};

#endif  // FEETECH_KALMAN_FILTER_HPP
