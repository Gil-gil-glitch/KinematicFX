package com.example.dhparameterfx;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link DifferentialDriveKinematics}, {@link Pose2D}, and
 * {@link WheelSpeeds}. These run headless — no JavaFX toolkit required —
 * which was the whole point of keeping the engine decoupled from the scene
 * graph. This is the automated counterpart to the ad-hoc
 * {@code MobileRobotTestApp.runSelfTest()} button: same checks, but runnable
 * in CI without launching the app.
 * <p>
 * Requires a JUnit 5 (Jupiter) dependency on the test classpath, e.g. in
 * Maven:
 * <pre>{@code
 * <dependency>
 *   <groupId>org.junit.jupiter</groupId>
 *   <artifactId>junit-jupiter</artifactId>
 *   <version>5.10.2</version>
 *   <scope>test</scope>
 * </dependency>
 * }</pre>
 */
class DifferentialDriveKinematicsTest {

    private static final double EPS = 1e-9;

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics();

    // A generously-capped chassis so tests exercise the math, not the clamp,
    // unless a test is specifically about clamping.
    private final DifferentialDriveKinematics.Chassis unclamped =
            new DifferentialDriveKinematics.Chassis(1.0, 3.0, 1000.0);

    // ---------------------------------------------------------------
    // Forward / inverse kinematics round-trip
    // ---------------------------------------------------------------

    @Test
    void inverseThenForward_recoversOriginalBodyVelocity() {
        double[][] samples = {
                {3.0, 0.0}, {0.0, 1.5}, {2.5, 0.8}, {-2.0, -0.5}, {5.0, 2.0}, {0.0, 0.0}
        };

        for (double[] s : samples) {
            double v = s[0], omega = s[1];
            WheelSpeeds wheels = kinematics.computeWheelSpeeds(v, omega, unclamped);
            DifferentialDriveKinematics.BodyVelocity recovered = kinematics.computeBodyVelocity(wheels, unclamped);

            assertEquals(v, recovered.v(), EPS, "linear velocity should round-trip for v=" + v + ", omega=" + omega);
            assertEquals(omega, recovered.omega(), EPS, "angular velocity should round-trip for v=" + v + ", omega=" + omega);
        }
    }

    @Test
    void equalWheelSpeeds_produceZeroAngularVelocity() {
        WheelSpeeds wheels = new WheelSpeeds(4.0, 4.0);
        DifferentialDriveKinematics.BodyVelocity bv = kinematics.computeBodyVelocity(wheels, unclamped);

        assertEquals(4.0, bv.v(), EPS);
        assertEquals(0.0, bv.omega(), EPS);
    }

    @Test
    void oppositeWheelSpeeds_produceZeroLinearVelocity() {
        WheelSpeeds wheels = new WheelSpeeds(-2.0, 2.0);
        DifferentialDriveKinematics.BodyVelocity bv = kinematics.computeBodyVelocity(wheels, unclamped);

        assertEquals(0.0, bv.v(), EPS, "spinning in place should have zero linear velocity");
        assertTrue(bv.omega() > 0, "right wheel forward / left wheel back should turn left (positive omega)");
    }

    // ---------------------------------------------------------------
    // Pose integration: straight line
    // ---------------------------------------------------------------

    @Test
    void straightLineIntegration_movesAlongHeadingWithNoDrift() {
        Pose2D start = new Pose2D(0, 0, 0);
        Pose2D result = kinematics.integrate(start, 5.0, 0.0, 2.0);

        assertEquals(10.0, result.x(), EPS);
        assertEquals(0.0, result.y(), EPS);
        assertEquals(0.0, result.thetaDeg(), EPS);
    }

    @Test
    void straightLineIntegration_atFortyFiveDegrees_movesDiagonally() {
        Pose2D start = new Pose2D(0, 0, 45.0);
        Pose2D result = kinematics.integrate(start, Math.sqrt(2.0), 0.0, 1.0);

        assertEquals(1.0, result.x(), 1e-6);
        assertEquals(1.0, result.y(), 1e-6);
        assertEquals(45.0, result.thetaDeg(), EPS);
    }

    // ---------------------------------------------------------------
    // Pose integration: pure rotation
    // ---------------------------------------------------------------

    @Test
    void pureRotation_neverMovesPosition() {
        Pose2D start = new Pose2D(2.0, -1.0, 30.0);
        Pose2D result = kinematics.integrate(start, 0.0, 1.2, 0.5);

        assertEquals(start.x(), result.x(), EPS, "v=0 must never change x");
        assertEquals(start.y(), result.y(), EPS, "v=0 must never change y");
        assertEquals(start.thetaDeg() + Math.toDegrees(1.2 * 0.5), result.thetaDeg(), 1e-6);
    }

    // ---------------------------------------------------------------
    // Pose integration: constant-curvature arc closes into a circle
    // ---------------------------------------------------------------

    @Test
    void constantCurvatureArc_returnsToStartAfterOneFullTurn() {
        double v = 3.0;
        double omega = 1.0; // rad/s
        double periodSeconds = 2 * Math.PI / omega;

        Pose2D pose = new Pose2D(5.0, -2.0, 17.0);
        Pose2D start = pose;

        // Integrate in small steps rather than one giant dt, the way the
        // animation loop actually calls integrate() every frame.
        int steps = 2000;
        double dt = periodSeconds / steps;
        for (int i = 0; i < steps; i++) {
            pose = kinematics.integrate(pose, v, omega, dt);
        }

        assertEquals(start.x(), pose.x(), 1e-3, "one full revolution should return to the start x");
        assertEquals(start.y(), pose.y(), 1e-3, "one full revolution should return to the start y");
        assertEquals(start.normalized().thetaDeg(), pose.normalized().thetaDeg(), 1e-2,
                "heading should also return to its starting value after one full turn");
    }

    @Test
    void constantCurvatureArc_matchesClosedFormRadius() {
        // For a unicycle driven at constant (v, omega), the path is a circle
        // of radius v/omega. Starting at the origin facing +X, after a
        // quarter turn the robot should be offset by exactly that radius in
        // both x and y (for omega > 0, turning left).
        double v = 4.0;
        double omega = 2.0;
        double radius = v / omega;

        Pose2D pose = Pose2D.origin();
        int steps = 2000;
        double quarterPeriod = (Math.PI / 2) / omega;
        double dt = quarterPeriod / steps;
        for (int i = 0; i < steps; i++) {
            pose = kinematics.integrate(pose, v, omega, dt);
        }

        assertEquals(radius, pose.x(), 1e-3);
        assertEquals(radius, pose.y(), 1e-3);
        assertEquals(90.0, pose.thetaDeg(), 1e-2);
    }

    // ---------------------------------------------------------------
    // Clamping
    // ---------------------------------------------------------------

    @Test
    void wheelSpeedsClamped_neverExceedMax() {
        WheelSpeeds fast = new WheelSpeeds(20.0, -15.0);
        WheelSpeeds clamped = fast.clamped(5.0);

        assertEquals(5.0, clamped.leftRadPerSec(), EPS);
        assertEquals(-5.0, clamped.rightRadPerSec(), EPS);
    }

    @Test
    void clampedWheelSpeeds_reduceAchievedVelocityWhenRequestExceedsLimit() {
        DifferentialDriveKinematics.Chassis tightChassis =
                new DifferentialDriveKinematics.Chassis(1.0, 3.0, 2.0); // max 2 rad/s per wheel

        // Requesting a large forward speed the chassis can't achieve.
        WheelSpeeds commanded = kinematics.computeClampedWheelSpeeds(10.0, 0.0, tightChassis);
        DifferentialDriveKinematics.BodyVelocity achieved = kinematics.computeBodyVelocity(commanded, tightChassis);

        assertTrue(achieved.v() < 10.0, "achieved v should be capped below the unreachable request");
        assertEquals(2.0, achieved.v(), EPS, "with both wheels driven equally, achieved v = maxWheelSpeed * wheelRadius");
        assertEquals(0.0, achieved.omega(), EPS, "a pure-forward request should still track straight after clamping");
    }

    @Test
    void chassisRejectsInvalidGeometry() {
        assertThrows(IllegalArgumentException.class, () ->
                new DifferentialDriveKinematics.Chassis(0.0, 3.0, 6.0));
        assertThrows(IllegalArgumentException.class, () ->
                new DifferentialDriveKinematics.Chassis(1.0, -1.0, 6.0));
        assertThrows(IllegalArgumentException.class, () ->
                new DifferentialDriveKinematics.Chassis(1.0, 3.0, 0.0));
    }

    // ---------------------------------------------------------------
    // Nonholonomic constraint helper
    // ---------------------------------------------------------------

    @Test
    void lateralMotionIsReportedAsUnachievable() {
        assertFalse(kinematics.isLateralMotionAchievable());
    }
}