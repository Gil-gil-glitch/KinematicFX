package com.example.dhparameterfx;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for {@link PurePursuitController}. These pin down the geometry
 * and sign conventions independently of the animation loop, and specifically
 * confirm the commands it produces are consistent with the same
 * left-is-positive-omega convention established in
 * {@link DifferentialDriveKinematicsTest}.
 */
class PurePursuitControllerTest {

    private final PurePursuitController controller = new PurePursuitController();

    @Test
    void straightAheadPath_producesZeroCurvature() {
        List<PurePursuitController.Waypoint> path = List.of(
                new PurePursuitController.Waypoint(5, 0),
                new PurePursuitController.Waypoint(10, 0));

        Pose2D pose = new Pose2D(0, 0, 0); // facing +x, path also runs along +x
        PurePursuitController.Command cmd = controller.computeCommand(pose, path, 0, 3.0, 4.0, 0.3);

        assertEquals(0.0, cmd.omega(), 1e-9, "a straight-ahead path should command zero turning");
        assertTrue(cmd.v() > 0, "should still command forward motion");
        assertFalse(cmd.pathComplete());
    }

    @Test
    void waypointToTheLeft_producesPositiveOmega() {
        // Robot facing +x, target directly to its left (+y in body frame,
        // which for theta=0 is also world +y).
        List<PurePursuitController.Waypoint> path = List.of(new PurePursuitController.Waypoint(5, 5));
        Pose2D pose = new Pose2D(0, 0, 0);

        PurePursuitController.Command cmd = controller.computeCommand(pose, path, 0, 3.0, 4.0, 0.3);

        assertTrue(cmd.omega() > 0,
                "a target to the left should command positive omega, matching the left-is-positive " +
                        "convention in DifferentialDriveKinematicsTest.oppositeWheelSpeeds_produceZeroLinearVelocity");
    }

    @Test
    void waypointToTheRight_producesNegativeOmega() {
        List<PurePursuitController.Waypoint> path = List.of(new PurePursuitController.Waypoint(5, -5));
        Pose2D pose = new Pose2D(0, 0, 0);

        PurePursuitController.Command cmd = controller.computeCommand(pose, path, 0, 3.0, 4.0, 0.3);

        assertTrue(cmd.omega() < 0, "a target to the right should command negative omega");
    }

    @Test
    void withinGoalTolerance_reportsPathCompleteAndZeroCommand() {
        List<PurePursuitController.Waypoint> path = List.of(new PurePursuitController.Waypoint(0.1, 0.05));
        Pose2D pose = new Pose2D(0, 0, 0);

        PurePursuitController.Command cmd = controller.computeCommand(pose, path, 0, 3.0, 4.0, 0.3);

        assertTrue(cmd.pathComplete());
        assertEquals(0.0, cmd.v(), 1e-9);
        assertEquals(0.0, cmd.omega(), 1e-9);
    }

    @Test
    void emptyPath_isImmediatelyComplete() {
        PurePursuitController.Command cmd = controller.computeCommand(
                Pose2D.origin(), List.of(), 0, 3.0, 4.0, 0.3);

        assertTrue(cmd.pathComplete());
    }

    @Test
    void targetIndexNeverMovesBackward() {
        List<PurePursuitController.Waypoint> path = List.of(
                new PurePursuitController.Waypoint(1, 0),
                new PurePursuitController.Waypoint(2, 0),
                new PurePursuitController.Waypoint(20, 0));

        // Robot already at the second waypoint, far from the first.
        Pose2D pose = new Pose2D(2, 0, 0);

        PurePursuitController.Command cmd = controller.computeCommand(pose, path, 1, 3.0, 4.0, 0.3);

        assertTrue(cmd.targetIndex() >= 1, "should never re-target an earlier waypoint than the given hint");
    }

    @Test
    void decelerationFactorApproachesZeroNearGoal() {
        List<PurePursuitController.Waypoint> path = List.of(new PurePursuitController.Waypoint(1.0, 0));
        Pose2D pose = new Pose2D(0, 0, 0);

        // Distance to goal (1.0) is much smaller than lookahead (10.0), so
        // the deceleration ramp should command a slow speed, not full speed.
        PurePursuitController.Command cmd = controller.computeCommand(pose, path, 0, 10.0, 4.0, 0.05);

        assertTrue(cmd.v() < 4.0, "speed should be reduced when close to the final waypoint");
        assertTrue(cmd.v() > 0.0, "but should not be exactly zero until within goalTolerance");
    }

    @Test
    void lookaheadSearchSkipsAlreadyReachedWaypoints() {
        List<PurePursuitController.Waypoint> path = List.of(
                new PurePursuitController.Waypoint(0.05, 0),   // essentially already reached
                new PurePursuitController.Waypoint(10, 0));

        Pose2D pose = new Pose2D(0, 0, 0);
        PurePursuitController.Command cmd = controller.computeCommand(pose, path, 0, 3.0, 4.0, 0.3);

        assertEquals(1, cmd.targetIndex(), "should skip the essentially-reached first waypoint and aim at the second");
    }
}