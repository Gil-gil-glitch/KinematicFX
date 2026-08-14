package com.example.dhparameterfx;

/**
 * Forward and inverse kinematics for a two-wheel differential-drive base,
 * plus exact (non-Euler) pose integration of the resulting unicycle model.
 * <p>
 * This class is intentionally free of JavaFX types so it can be unit tested
 * directly, the same way {@link ForwardKinematicsEngine} and {@link IKSolver}
 * are decoupled from the scene graph. {@link MobileRobotModel} is the
 * JavaFX-property-backed wrapper meant for UI binding.
 * <p>
 * Sign / frame convention: body-frame +x is "forward", +y is "left", and a
 * positive angular velocity omega is a counter-clockwise (left) turn, i.e.
 * the same right-handed convention as {@code Rotate.Z_AXIS} rotations
 * elsewhere in the app. Theta is tracked in degrees to match the rest of the
 * codebase; this class converts to radians internally wherever trig is
 * required.
 */
public class DifferentialDriveKinematics {

    /** Physical robot parameters. Immutable; build a new instance if geometry changes. */
    public record Chassis(double wheelRadius, double trackWidth, double maxWheelSpeedRadPerSec) {
        public Chassis {
            if (wheelRadius <= 0) throw new IllegalArgumentException("wheelRadius must be > 0");
            if (trackWidth <= 0) throw new IllegalArgumentException("trackWidth must be > 0");
            if (maxWheelSpeedRadPerSec <= 0) throw new IllegalArgumentException("maxWheelSpeedRadPerSec must be > 0");
        }
    }

    /** Body-frame velocity of a unicycle-equivalent robot: linear speed v (units/s) and turn rate omega (rad/s). */
    public record BodyVelocity(double v, double omega) {
        public static BodyVelocity zero() {
            return new BodyVelocity(0.0, 0.0);
        }
    }

    // Forward kinematics: wheel speeds -> body velocity
    /**
     * Standard differential-drive forward kinematics.
     * v     = r * (omegaR + omegaL) / 2
     * omega = r * (omegaR - omegaL) / L
     */
    public BodyVelocity computeBodyVelocity(WheelSpeeds wheels, Chassis chassis) {
        double r = chassis.wheelRadius();
        double L = chassis.trackWidth();
        double v = r * (wheels.rightRadPerSec() + wheels.leftRadPerSec()) / 2.0;
        double omega = r * (wheels.rightRadPerSec() - wheels.leftRadPerSec()) / L;
        return new BodyVelocity(v, omega);
    }

    // Inverse kinematics: desired body velocity -> wheel speeds
    /**
     * Inverts the forward kinematics above:
     * omegaL = (v - omega*L/2) / r
     * omegaR = (v + omega*L/2) / r
     * <p>
     * NOTE: unlike the arm's IKSolver, this is a closed-form (non-iterative)
     * inversion — differential drive is not kinematically redundant, so
     * every achievable (v, omega) maps to exactly one wheel-speed pair.
     */
    public WheelSpeeds computeWheelSpeeds(double v, double omega, Chassis chassis) {
        double r = chassis.wheelRadius();
        double halfL = chassis.trackWidth() / 2.0;
        double omegaL = (v - omega * halfL) / r;
        double omegaR = (v + omega * halfL) / r;
        return new WheelSpeeds(omegaL, omegaR);
    }

    /**
     * Requests a body velocity, clamps the resulting wheel speeds to the
     * chassis's max wheel speed, and returns the wheel speeds that were
     * actually commanded. Use {@link #computeBodyVelocity} on the result to
     * find out what (v, omega) is actually achieved after clamping — this
     * mirrors the "attempt the move, then report what was actually reached"
     * pattern used by {@code IKSolver.solve}.
     */
    public WheelSpeeds computeClampedWheelSpeeds(double v, double omega, Chassis chassis) {
        return computeWheelSpeeds(v, omega, chassis).clamped(chassis.maxWheelSpeedRadPerSec());
    }

    // Pose integration (unicycle model, exact arc solution)
    private static final double OMEGA_EPSILON = 1e-6;

    /**
     * Integrates the unicycle model exactly over dt seconds, rather than
     * with a first-order Euler step, so a constant (v, omega) command
     * traces a true circular arc with no accumulated curvature error.
     * <p>
     * Straight-line case (|omega| ~ 0):
     *   x' = x + v*dt*cos(theta),  y' = y + v*dt*sin(theta),  theta' = theta
     * <p>
     * Constant-curvature case:
     *   theta' = theta + omega*dt
     *   x'     = x + (v/omega) * (sin(theta') - sin(theta))
     *   y'     = y - (v/omega) * (cos(theta') - cos(theta))
     * <p>
     * This is also the closed-form the self-test / HUD should check against
     * when validating straight-line and pure-rotation motion, analogous to
     * how {@code runFullChainSelfTest} checks the arm's step transform
     * against known elementary-matrix candidates.
     */
    public Pose2D integrate(Pose2D pose, double v, double omega, double dt) {
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

    /**
     * Convenience overload: integrates directly from wheel speeds, going
     * through {@link #computeBodyVelocity} first. Useful when driving the
     * simulation from raw encoder/commanded wheel rates.
     */
    public Pose2D integrateFromWheelSpeeds(Pose2D pose, WheelSpeeds wheels, Chassis chassis, double dt) {
        BodyVelocity bv = computeBodyVelocity(wheels, chassis);
        return integrate(pose, bv.v(), bv.omega(), dt);
    }

    // Nonholonomic constraint helper
    /**
     * A differential-drive (or Ackermann) base can never realize instantaneous
     * lateral (sideways, body-frame +y) motion — that is precisely what
     * "nonholonomic" means for this platform. There is no wheel configuration
     * that produces it, so unlike the arm's {@code IKSolver.isTargetReachable}
     * this is not a distance check: any requested body-frame y-velocity is
     * simply unrealizable and should be reported/blocked at the UI layer
     * (e.g. disable a "strafe" slider) rather than solved for.
     *
     * @return true always, for this drive type — included so UI code has a
     *         single, self-documenting call site to gate strafe-style
     *         controls, and so a future OmniKinematics/AckermannKinematics
     *         class can override with real per-platform logic.
     */
    public boolean isLateralMotionAchievable() {
        return false;

    }
}