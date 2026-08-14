package com.example.dhparameterfx;


/**
 * Planar robot pose: position (x, y) and heading theta.
 * <p>
 * Theta is stored in DEGREES, matching the convention used throughout the
 * existing project ({@link DHParameterModel#getTheta()},
 * {@link DHParameterModel#getAlpha()}), so it can be dropped into the same
 * TextField/Slider/RotaryDial UI patterns without a unit-mismatch bug.
 * x/y are in the same scene units as the arm's a/d parameters.
 */
public record Pose2D(double x, double y, double thetaDeg) {

    public static Pose2D origin() {
        return new Pose2D(0.0, 0.0, 0.0);
    }

    /** Heading in radians, for trig. */
    public double thetaRad() {
        return Math.toRadians(thetaDeg);
    }

    /** Returns a copy with theta normalized to (-180, 180]. */
    public Pose2D normalized() {
        double t = thetaDeg % 360.0;
        if (t <= -180.0) t += 360.0;
        if (t > 180.0) t -= 360.0;
        return new Pose2D(x, y, t);
    }
}