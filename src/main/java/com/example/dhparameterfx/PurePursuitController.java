package com.example.dhparameterfx;

import java.util.List;

/**
 * Curvature-based path follower (pure pursuit) for a unicycle-model robot.
 * <p>
 * This is the answer to the "is {@code TrajectoryPlanner.interpolateCubic}
 * reusable, or is a curvature-constrained planner needed" investigation:
 * naively cubic-interpolating (x, y) between waypoints can demand
 * instantaneous lateral velocity to stay on the interpolated curve, which a
 * differential-drive base cannot produce (see
 * {@link DifferentialDriveKinematics#isLateralMotionAchievable}). Pure
 * pursuit sidesteps that by construction: it never computes a desired
 * pose to snap to, only a required curvature from current pose to a
 * lookahead point, then converts that directly into {@code omega = curvature
 * * v}. Every command it emits is something a unicycle model can actually
 * execute — there is no "infeasible" output to clamp after the fact.
 * <p>
 * Geometry (classic pure pursuit): transform the lookahead point into the
 * robot's body frame; if it lands at body-frame offset (x_l, y_l), the arc
 * that passes through the origin (heading along +x) and that point has
 * curvature kappa = 2 * y_l / (x_l^2 + y_l^2). Command omega = kappa * v.
 * <p>
 * Like {@link DifferentialDriveKinematics}, this class is free of JavaFX
 * types so it can be unit tested directly. It advances an index into the
 * waypoint list rather than owning any mutable path state itself, so the
 * caller (e.g. {@code MobileRobotTestApp}) decides how/when a path is
 * loaded, extended, or cleared.
 */
public class PurePursuitController {

    /** A single 2D path waypoint, in the same (x, y) units as {@link Pose2D}. */
    public record Waypoint(double x, double y) {
    }

    /**
     * @param v            commanded linear speed (units/s)
     * @param omega        commanded angular speed (rad/s)
     * @param targetIndex  index into the path this command aimed at — pass
     *                     this back in as {@code startIndex} on the next
     *                     call so the search never walks backward
     * @param pathComplete true once the robot is within goalTolerance of the
     *                     final waypoint; v and omega are both 0 in that case
     */
    public record Command(double v, double omega, int targetIndex, boolean pathComplete) {
    }

    private static final double MIN_LOOKAHEAD_DENOMINATOR = 1e-6;

    /**
     * Computes the next (v, omega) command to follow {@code path}.
     *
     * @param pose              current robot pose
     * @param path              ordered waypoints; must not be empty
     * @param startIndex        index to resume searching from (0 on first call)
     * @param lookaheadDistance how far ahead along the path to aim; larger
     *                          values produce smoother but less precise
     *                          cornering, smaller values hug the path more
     *                          tightly but can oscillate
     * @param desiredSpeed      linear speed to command when not decelerating
     *                          into the goal
     * @param goalTolerance     distance from the final waypoint at which the
     *                          path is considered complete
     */
    public Command computeCommand(Pose2D pose, List<Waypoint> path, int startIndex,
                                  double lookaheadDistance, double desiredSpeed, double goalTolerance) {
        if (path == null || path.isEmpty()) {
            return new Command(0.0, 0.0, 0, true);
        }

        int lastIndex = path.size() - 1;
        Waypoint goal = path.get(lastIndex);
        double distToGoal = Math.hypot(goal.x() - pose.x(), goal.y() - pose.y());

        if (distToGoal <= goalTolerance) {
            return new Command(0.0, 0.0, lastIndex, true);
        }

        // Walk forward (never backward) to the first waypoint at or beyond
        // lookaheadDistance, stopping at the last waypoint if the path runs
        // out first. This is what lets waypoints already "passed" be
        // skipped without the controller ever un-advancing its progress.
        int targetIndex = Math.max(0, Math.min(startIndex, lastIndex));
        while (targetIndex < lastIndex
                && distanceTo(pose, path.get(targetIndex)) < lookaheadDistance) {
            targetIndex++;
        }

        Waypoint target = path.get(targetIndex);
        double dx = target.x() - pose.x();
        double dy = target.y() - pose.y();

        double cos = Math.cos(pose.thetaRad());
        double sin = Math.sin(pose.thetaRad());

        // World -> body frame rotation by -theta. Body +x is forward, body
        // +y is left, matching the convention documented on
        // DifferentialDriveKinematics.
        double localX = dx * cos + dy * sin;
        double localY = -dx * sin + dy * cos;

        double lookaheadSq = localX * localX + localY * localY;
        double curvature = lookaheadSq < MIN_LOOKAHEAD_DENOMINATOR
                ? 0.0
                : (2.0 * localY) / lookaheadSq;

        // Slow down approaching the final waypoint rather than arriving at
        // full speed and overshooting; ramps linearly over one lookahead
        // distance's worth of remaining travel.
        double decelFactor = Math.min(1.0, distToGoal / Math.max(lookaheadDistance, 1e-6));
        double v = desiredSpeed * decelFactor;
        double omega = curvature * v;

        return new Command(v, omega, targetIndex, false);
    }

    private double distanceTo(Pose2D pose, Waypoint w) {
        return Math.hypot(w.x() - pose.x(), w.y() - pose.y());
    }
}