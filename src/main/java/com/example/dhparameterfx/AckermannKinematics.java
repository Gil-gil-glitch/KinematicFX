package com.example.dhparameterfx;

/**
 * Forward and inverse kinematics for an Ackermann-steered (car-like) base,
 * using the standard bicycle-model simplification for planar motion plus
 * real per-wheel Ackermann steering-angle geometry for visualization.
 * <p>
 * Structurally this mirrors {@link DifferentialDriveKinematics} — same
 * Chassis/BodyVelocity record shape, same JavaFX-free unit-testability, same
 * body-frame sign convention (+x forward, +y left, positive omega = left
 * turn) — but the control input is fundamentally different. A differential
 * drive is controlled via two independent wheel speeds and can realize any
 * (v, omega) pair (subject only to a wheel-speed magnitude cap). An
 * Ackermann base is controlled via a single speed v and a single front
 * steering angle delta, and:
 * <ul>
 *   <li>the achievable curvature is bounded by {@code tan(maxSteeringAngle) / wheelbase},
 *       regardless of how much torque or speed is available — unlike a wheel-speed
 *       cap, this is a hard geometric ceiling on turning tightness;</li>
 *   <li>at v = 0, no steering angle produces any turning at all — the
 *       vehicle <em>cannot pivot in place</em>, which is a categorically
 *       different nonholonomic constraint than differential drive's "no
 *       lateral motion" (see {@link #isPivotTurnAchievable}).</li>
 * </ul>
 */
public class AckermannKinematics {

    private static final double EPS = 1e-9;

    /** Physical robot parameters. Immutable; build a new instance if geometry changes. */
    public record Chassis(double wheelbase, double trackWidth, double maxSteeringAngleRad, double maxSpeed) {
        public Chassis {
            if (wheelbase <= 0) throw new IllegalArgumentException("wheelbase must be > 0");
            if (trackWidth <= 0) throw new IllegalArgumentException("trackWidth must be > 0");
            if (maxSteeringAngleRad <= 0 || maxSteeringAngleRad >= Math.PI / 2) {
                throw new IllegalArgumentException("maxSteeringAngleRad must be in (0, PI/2)");
            }
            if (maxSpeed <= 0) throw new IllegalArgumentException("maxSpeed must be > 0");
        }

        /**
         * Tightest turning radius this chassis can achieve, at any speed —
         * curvature for Ackermann depends only on the steering angle and
         * wheelbase, never on how fast the vehicle is moving.
         */
        public double minTurningRadius() {
            return wheelbase / Math.tan(maxSteeringAngleRad);
        }
    }

    /** Body-frame velocity of the equivalent unicycle: linear speed v (units/s) and turn rate omega (rad/s). */
    public record BodyVelocity(double v, double omega) {
        public static BodyVelocity zero() {
            return new BodyVelocity(0.0, 0.0);
        }
    }

    /** Per-wheel steering angles for the two front (steered) wheels, accounting for Ackermann geometry. */
    public record FrontWheelAngles(double leftRad, double rightRad) {
    }

    // ---------------------------------------------------------------
    // Forward kinematics: (v, steering angle) -> body velocity
    // ---------------------------------------------------------------

    /**
     * Bicycle-model forward kinematics, reference point at the rear axle
     * center: omega = v * tan(delta) / wheelbase. Curvature (omega / v) is
     * independent of speed — it depends only on the steering angle and
     * wheelbase, which is the geometric ceiling {@link Chassis#minTurningRadius}
     * describes.
     */
    public BodyVelocity computeBodyVelocity(double v, double steeringAngleRad, Chassis chassis) {
        double omega = v * Math.tan(steeringAngleRad) / chassis.wheelbase();
        return new BodyVelocity(v, omega);
    }

    // ---------------------------------------------------------------
    // Inverse kinematics: desired body velocity -> steering angle
    // ---------------------------------------------------------------

    /**
     * Inverts the forward kinematics above: delta = atan(omega * wheelbase / v),
     * clamped to the chassis's max steering angle.
     * <p>
     * Unlike {@code DifferentialDriveKinematics.computeWheelSpeeds}, this
     * inversion has a real singularity, not just a hardware limit: at
     * v = 0 there is no finite delta that produces nonzero omega (dividing
     * by v blows up), because an Ackermann vehicle genuinely cannot turn
     * in place. This method returns 0 in that case rather than NaN/infinity
     * — callers that need to know whether a turn-in-place was actually
     * requested (and silently dropped) should check
     * {@link #isPivotTurnAchievable} themselves rather than infer it from a
     * 0 steering angle, since 0 is also the correct answer for "drive
     * straight."
     */
    public double computeSteeringAngle(double v, double omega, Chassis chassis) {
        if (Math.abs(v) < EPS) {
            return 0.0;
        }
        double delta = Math.atan(omega * chassis.wheelbase() / v);
        return clampSteeringAngle(delta, chassis);
    }

    public double clampSteeringAngle(double steeringAngleRad, Chassis chassis) {
        double max = chassis.maxSteeringAngleRad();
        return Math.max(-max, Math.min(max, steeringAngleRad));
    }

    /**
     * Requests a body velocity, clamps the resulting steering angle to the
     * chassis's limit, and returns the body velocity actually achieved —
     * mirroring {@code DifferentialDriveKinematics.computeClampedWheelSpeeds}'s
     * "attempt the move, report what was actually reached" pattern. Speed
     * itself is also clamped to {@code chassis.maxSpeed()}.
     */
    public BodyVelocity computeClampedBodyVelocity(double v, double omega, Chassis chassis) {
        double clampedV = Math.max(-chassis.maxSpeed(), Math.min(chassis.maxSpeed(), v));
        double delta = computeSteeringAngle(clampedV, omega, chassis);
        return computeBodyVelocity(clampedV, delta, chassis);
    }

    // ---------------------------------------------------------------
    // Per-wheel Ackermann steering geometry (for visualization)
    // ---------------------------------------------------------------

    /**
     * Computes the two front wheels' individual steering angles so they
     * trace concentric circles about a common instantaneous center of
     * rotation, rather than both pointing the bicycle-model's single
     * "effective" angle delta — real Ackermann linkages toe the inner wheel
     * in more sharply than the outer wheel specifically to avoid tire
     * scrub. Derived from the turning radius R = wheelbase / tan(delta)
     * without ever computing R explicitly (it's infinite at delta = 0),
     * so both outputs go cleanly to 0 as delta -> 0:
     * <pre>
     *   tan(delta_left)  = tan(delta) / (1 - (trackWidth * tan(delta)) / (2 * wheelbase))
     *   tan(delta_right) = tan(delta) / (1 + (trackWidth * tan(delta)) / (2 * wheelbase))
     * </pre>
     * For a left turn (delta &gt; 0, matching this class's left-is-positive
     * convention) the left wheel is the inner wheel and ends up with the
     * larger-magnitude angle; for a right turn the roles swap automatically
     * through the sign of tan(delta), with no separate branching needed.
     */
    public FrontWheelAngles computeAckermannWheelAngles(double steeringAngleRad, Chassis chassis) {
        double t = Math.tan(steeringAngleRad);
        double halfTrackOverWheelbase = chassis.trackWidth() * t / (2.0 * chassis.wheelbase());

        double denomLeft = 1.0 - halfTrackOverWheelbase;
        double denomRight = 1.0 + halfTrackOverWheelbase;

        double leftAngle = Math.abs(denomLeft) < EPS
                ? Math.copySign(Math.PI / 2, t)
                : Math.atan(t / denomLeft);
        double rightAngle = Math.abs(denomRight) < EPS
                ? Math.copySign(Math.PI / 2, t)
                : Math.atan(t / denomRight);

        return new FrontWheelAngles(leftAngle, rightAngle);
    }

    // ---------------------------------------------------------------
    // Pose integration — identical unicycle-model math to differential
    // drive once (v, omega) is known; see UnicyclePoseIntegrator.
    // ---------------------------------------------------------------

    public Pose2D integrate(Pose2D pose, double v, double omega, double dt) {
        return UnicyclePoseIntegrator.integrate(pose, v, omega, dt);
    }

    // ---------------------------------------------------------------
    // Nonholonomic constraint helper
    // ---------------------------------------------------------------

    /**
     * An Ackermann-steered vehicle cannot rotate in place: at v = 0, every
     * steering angle produces omega = 0 (see {@link #computeBodyVelocity}),
     * so turning always requires simultaneous translation. This is a
     * different nonholonomic constraint than differential drive's "no
     * lateral motion" ({@link DifferentialDriveKinematics#isLateralMotionAchievable}) —
     * a diff-drive base can spin in place but never slide sideways; an
     * Ackermann base can never spin in place at all.
     *
     * @return false always, for this drive type.
     */
    public boolean isPivotTurnAchievable() {
        return false;
    }
}