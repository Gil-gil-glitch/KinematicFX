package com.example.dhparameterfx;

/**
 * Exact (non-Euler) pose integration for the unicycle model: given a
 * body-frame linear speed v and angular speed omega, advances a
 * {@link Pose2D} by dt seconds along the true circular arc that (v, omega)
 * describes, rather than approximating it with a straight-line step.
 * <p>
 * This is deliberately factored out of {@link DifferentialDriveKinematics}:
 * once you have (v, omega), the pose kinematics
 * (x-dot = v cos(theta), y-dot = v sin(theta), theta-dot = omega) are
 * <em>identical</em> regardless of what kind of chassis produced that
 * (v, omega) — a differential-drive base derives it from two wheel speeds,
 * an Ackermann base derives it from a speed and a steering angle, but both
 * then evolve pose exactly the same way. Sharing this one implementation
 * means a fix to the arc math (or its zero-omega edge case) only has to
 * happen in one place, rather than needing to stay in sync across every
 * drive-type class.
 */
public final class UnicyclePoseIntegrator {

    private static final double OMEGA_EPSILON = 1e-6;

    private UnicyclePoseIntegrator() {
    }

    /**
     * Straight-line case (|omega| ~ 0):
     *   x' = x + v*dt*cos(theta),  y' = y + v*dt*sin(theta),  theta' = theta
     * <p>
     * Constant-curvature case:
     *   theta' = theta + omega*dt
     *   x'     = x + (v/omega) * (sin(theta') - sin(theta))
     *   y'     = y - (v/omega) * (cos(theta') - cos(theta))
     */
    public static Pose2D integrate(Pose2D pose, double v, double omega, double dt) {
        double theta = pose.thetaRad();

        if (Math.abs(omega) < OMEGA_EPSILON) {
            double x = pose.x() + v * dt * Math.cos(theta);
            double y = pose.y() + v * dt * Math.sin(theta);
            return new Pose2D(x, y, pose.thetaDeg());
        }

        double thetaNext = theta + omega * dt;
        double x = pose.x() + (v / omega) * (Math.sin(thetaNext) - Math.sin(theta));
        double y = pose.y() - (v / omega) * (Math.cos(thetaNext) - Math.cos(theta));
        return new Pose2D(x, y, Math.toDegrees(thetaNext));
    }
}