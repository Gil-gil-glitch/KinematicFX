package com.example.dhparameterfx;

/**
 * Common surface shared by every planar mobile-robot model
 * ({@link MobileRobotModel} for differential drive, {@link AckermannRobotModel}
 * for Ackermann steering, and any future drive type), so the app's
 * animation loop, HUD, path-follower, and camera-tracking code can all work
 * against one type instead of branching on drive type at every call site.
 * <p>
 * Each implementation is still free to expose its own drive-specific
 * properties (wheel speeds for diff-drive, steering angle for Ackermann) for
 * its own control-panel section and HUD line — this interface only covers
 * what every planar model has in common: a pose, and a way to advance it
 * from a requested body velocity.
 */
public interface PlanarRobotModel {

    Pose2D getPose();

    void setPose(Pose2D pose);

    /**
     * Advances the model by dt seconds toward the requested body velocity,
     * clamping to whatever hardware limits that drive type has (wheel-speed
     * cap for diff-drive, steering-angle and speed caps for Ackermann) and
     * integrating the pose from whatever velocity was actually achievable
     * after clamping — never from the raw request.
     */
    void step(double requestedV, double requestedOmega, double dt);
}