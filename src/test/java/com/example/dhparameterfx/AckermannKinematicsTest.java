package com.example.dhparameterfx;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link AckermannKinematics}. Mirrors the structure of
 * {@link DifferentialDriveKinematicsTest} where the underlying question is
 * the same (does FK/IK round-trip, does the arc close correctly), and adds
 * tests specific to what actually differs about Ackermann: the v=0
 * singularity, steering-angle clamping, and per-wheel toe-in geometry.
 */
class AckermannKinematicsTest {

    private static final double EPS = 1e-9;

    private final AckermannKinematics kinematics = new AckermannKinematics();

    // Roughly car-like proportions; generous steering limit so most test
    // angles aren't clipped unless a test is specifically about clamping.
    private final AckermannKinematics.Chassis chassis =
            new AckermannKinematics.Chassis(2.5, 1.6, Math.toRadians(35), 20.0);

    // ---------------------------------------------------------------
    // Forward / inverse kinematics
    // ---------------------------------------------------------------

    @Test
    void zeroSteeringAngle_producesZeroOmega() {
        AckermannKinematics.BodyVelocity bv = kinematics.computeBodyVelocity(5.0, 0.0, chassis);
        assertEquals(5.0, bv.v(), EPS);
        assertEquals(0.0, bv.omega(), EPS);
    }

    @Test
    void positiveSteeringAngle_producesPositiveOmega_matchingLeftIsPositiveConvention() {
        AckermannKinematics.BodyVelocity bv = kinematics.computeBodyVelocity(5.0, Math.toRadians(10), chassis);
        assertTrue(bv.omega() > 0, "a positive (left) steering angle should turn left, i.e. positive omega, " +
                "matching the convention in DifferentialDriveKinematicsTest");
    }

    @Test
    void inverseThenForward_recoversRequestedOmega_whenWithinSteeringLimit() {
        double v = 6.0;
        double[] requestedOmegas = {0.0, 0.5, -0.5, 1.0, -1.0};

        for (double omega : requestedOmegas) {
            double delta = kinematics.computeSteeringAngle(v, omega, chassis);
            AckermannKinematics.BodyVelocity recovered = kinematics.computeBodyVelocity(v, delta, chassis);
            assertEquals(omega, recovered.omega(), 1e-6,
                    "omega=" + omega + " should round-trip through steering angle when within the limit");
        }
    }

    // ---------------------------------------------------------------
    // The v=0 pivot-turn singularity — the defining Ackermann constraint
    // ---------------------------------------------------------------

    @Test
    void atZeroSpeed_steeringAngleIsZero_regardlessOfRequestedOmega() {
        // No finite steering angle can produce nonzero omega at v=0 — this
        // is what makes Ackermann unable to pivot in place, unlike
        // differential drive.
        assertEquals(0.0, kinematics.computeSteeringAngle(0.0, 2.0, chassis), EPS);
        assertEquals(0.0, kinematics.computeSteeringAngle(0.0, -2.0, chassis), EPS);
    }

    @Test
    void pivotTurnIsReportedAsUnachievable() {
        assertFalse(kinematics.isPivotTurnAchievable());
    }

    // ---------------------------------------------------------------
    // Steering angle clamping and the geometric turning-radius ceiling
    // ---------------------------------------------------------------

    @Test
    void steeringAngleClampsToChassisLimit() {
        double huge = kinematics.computeSteeringAngle(5.0, 100.0, chassis); // would require an enormous angle
        assertEquals(chassis.maxSteeringAngleRad(), huge, EPS);

        double hugeNegative = kinematics.computeSteeringAngle(5.0, -100.0, chassis);
        assertEquals(-chassis.maxSteeringAngleRad(), hugeNegative, EPS);
    }

    @Test
    void minTurningRadius_matchesMaxSteeringAngle() {
        double expected = chassis.wheelbase() / Math.tan(chassis.maxSteeringAngleRad());
        assertEquals(expected, chassis.minTurningRadius(), 1e-9);
    }

    @Test
    void curvatureCeiling_isIndependentOfSpeed() {
        // Unlike a wheel-speed cap (which scales with speed), the maximum
        // achievable curvature (omega / v) is the same at any speed for
        // Ackermann, since it depends only on the steering limit and
        // wheelbase.
        double maxCurvature = Math.tan(chassis.maxSteeringAngleRad()) / chassis.wheelbase();

        for (double v : new double[]{1.0, 5.0, 15.0}) {
            AckermannKinematics.BodyVelocity bv =
                    kinematics.computeBodyVelocity(v, chassis.maxSteeringAngleRad(), chassis);
            assertEquals(maxCurvature, bv.omega() / v, 1e-9,
                    "curvature at max steering angle should be identical regardless of speed");
        }
    }

    @Test
    void clampedBodyVelocity_respectsBothSpeedAndSteeringLimits() {
        AckermannKinematics.BodyVelocity bv = kinematics.computeClampedBodyVelocity(1000.0, 1000.0, chassis);
        assertEquals(chassis.maxSpeed(), bv.v(), EPS);

        double maxCurvature = Math.tan(chassis.maxSteeringAngleRad()) / chassis.wheelbase();
        assertEquals(maxCurvature, bv.omega() / bv.v(), 1e-6);
    }

    // ---------------------------------------------------------------
    // Per-wheel Ackermann geometry
    // ---------------------------------------------------------------

    @Test
    void zeroSteeringAngle_bothFrontWheelsPointStraight() {
        AckermannKinematics.FrontWheelAngles angles = kinematics.computeAckermannWheelAngles(0.0, chassis);
        assertEquals(0.0, angles.leftRad(), EPS);
        assertEquals(0.0, angles.rightRad(), EPS);
    }

    @Test
    void leftTurn_innerWheelIsLeftAndTurnsMoreSharply() {
        double delta = Math.toRadians(20); // positive = left turn
        AckermannKinematics.FrontWheelAngles angles = kinematics.computeAckermannWheelAngles(delta, chassis);

        assertTrue(angles.leftRad() > angles.rightRad(),
                "on a left turn the left (inner) wheel should be steered more sharply than the right (outer) wheel");
        assertTrue(angles.leftRad() > delta, "the inner wheel's angle should exceed the bicycle-model average angle");
        assertTrue(angles.rightRad() < delta, "the outer wheel's angle should be less than the bicycle-model average angle");
    }

    @Test
    void rightTurn_innerOuterRolesSwapAutomatically() {
        double delta = Math.toRadians(-20); // negative = right turn
        AckermannKinematics.FrontWheelAngles angles = kinematics.computeAckermannWheelAngles(delta, chassis);

        assertTrue(angles.rightRad() < angles.leftRad(),
                "on a right turn the right (inner) wheel should be steered more sharply (more negative) than the left");
    }

    // ---------------------------------------------------------------
    // Pose integration — shared math with DifferentialDriveKinematics
    // ---------------------------------------------------------------

    @Test
    void constantCurvatureArc_returnsToStartAfterOneFullTurn() {
        double v = 4.0;
        double delta = Math.toRadians(15);
        AckermannKinematics.BodyVelocity bv = kinematics.computeBodyVelocity(v, delta, chassis);

        double period = 2 * Math.PI / bv.omega();
        Pose2D pose = new Pose2D(3.0, -1.0, 40.0);
        Pose2D start = pose;

        int steps = 2000;
        double dt = period / steps;
        for (int i = 0; i < steps; i++) {
            pose = kinematics.integrate(pose, bv.v(), bv.omega(), dt);
        }

        assertEquals(start.x(), pose.x(), 1e-3);
        assertEquals(start.y(), pose.y(), 1e-3);
    }

    @Test
    void straightLineIntegration_matchesDifferentialDriveForSameVAndOmega() {
        // With omega=0, pose integration must be identical between drive
        // types, since both delegate to the same UnicyclePoseIntegrator.
        DifferentialDriveKinematics diffDrive = new DifferentialDriveKinematics();
        Pose2D start = new Pose2D(1.0, 2.0, 25.0);

        Pose2D ackermannResult = kinematics.integrate(start, 3.0, 0.0, 1.5);
        Pose2D diffDriveResult = diffDrive.integrate(start, 3.0, 0.0, 1.5);

        assertEquals(diffDriveResult.x(), ackermannResult.x(), EPS);
        assertEquals(diffDriveResult.y(), ackermannResult.y(), EPS);
        assertEquals(diffDriveResult.thetaDeg(), ackermannResult.thetaDeg(), EPS);
    }

    // ---------------------------------------------------------------
    // Chassis validation
    // ---------------------------------------------------------------

    @Test
    void chassisRejectsInvalidGeometry() {
        assertThrows(IllegalArgumentException.class, () ->
                new AckermannKinematics.Chassis(0.0, 1.6, Math.toRadians(35), 20.0));
        assertThrows(IllegalArgumentException.class, () ->
                new AckermannKinematics.Chassis(2.5, -1.0, Math.toRadians(35), 20.0));
        assertThrows(IllegalArgumentException.class, () ->
                new AckermannKinematics.Chassis(2.5, 1.6, 0.0, 20.0));
        assertThrows(IllegalArgumentException.class, () ->
                new AckermannKinematics.Chassis(2.5, 1.6, Math.toRadians(95), 20.0));
        assertThrows(IllegalArgumentException.class, () ->
                new AckermannKinematics.Chassis(2.5, 1.6, Math.toRadians(35), 0.0));
    }
}