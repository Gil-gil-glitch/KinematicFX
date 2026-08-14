package com.example.dhparameterfx;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;

/**
 * JavaFX-property-backed differential-drive robot state: chassis geometry,
 * current pose, and current wheel speeds. This is the mobile-robot analogue
 * of {@link DHParameterModel} — a plain observable data holder that a future
 * control panel (Slider/TextField rows, RotaryDial-style widgets) can bind
 * to directly, while the actual math stays in {@link DifferentialDriveKinematics}.
 * <p>
 * This class does not touch the scene graph. Rendering (chassis box, wheel
 * cylinders, heading indicator) and the click-to-drive / teleop control
 * panel are follow-up work once this core engine is in place.
 */
public class MobileRobotModel {

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics();

    // Chassis geometry
    private final DoubleProperty wheelRadius = new SimpleDoubleProperty(1.0);
    private final DoubleProperty trackWidth = new SimpleDoubleProperty(3.0);
    private final DoubleProperty maxWheelSpeedRadPerSec = new SimpleDoubleProperty(10.0);

    // Pose
    private final DoubleProperty x = new SimpleDoubleProperty(0.0);
    private final DoubleProperty y = new SimpleDoubleProperty(0.0);
    private final DoubleProperty thetaDeg = new SimpleDoubleProperty(0.0);

    // Last commanded/achieved wheel speeds, exposed for HUD/debug display
    private final DoubleProperty leftWheelSpeed = new SimpleDoubleProperty(0.0);
    private final DoubleProperty rightWheelSpeed = new SimpleDoubleProperty(0.0);

    public MobileRobotModel() {
    }

    public MobileRobotModel(double wheelRadius, double trackWidth, double maxWheelSpeedRadPerSec) {
        this.wheelRadius.set(wheelRadius);
        this.trackWidth.set(trackWidth);
        this.maxWheelSpeedRadPerSec.set(maxWheelSpeedRadPerSec);
    }

    private DifferentialDriveKinematics.Chassis chassis() {
        return new DifferentialDriveKinematics.Chassis(
                wheelRadius.get(), trackWidth.get(), maxWheelSpeedRadPerSec.get());
    }

    public Pose2D getPose() {
        return new Pose2D(x.get(), y.get(), thetaDeg.get());
    }

    public void setPose(Pose2D pose) {
        x.set(pose.x());
        y.set(pose.y());
        thetaDeg.set(pose.thetaDeg());
    }

    /**
     * Advances the simulation by dt seconds toward the requested body
     * velocity (v units/s, omega rad/s). Wheel speeds are clamped to
     * {@code maxWheelSpeedRadPerSec}, then the pose is integrated using the
     * velocity that was actually achievable after clamping — so, like
     * {@code runIKAndAnimate}'s per-frame reachability handling for the arm,
     * a request that exceeds hardware limits degrades gracefully instead of
     * teleporting or diverging.
     *
     * @return the body velocity actually achieved this step, after clamping
     */
    public DifferentialDriveKinematics.BodyVelocity step(double requestedV, double requestedOmega, double dt) {
        DifferentialDriveKinematics.Chassis c = chassis();

        WheelSpeeds commanded = kinematics.computeClampedWheelSpeeds(requestedV, requestedOmega, c);
        leftWheelSpeed.set(commanded.leftRadPerSec());
        rightWheelSpeed.set(commanded.rightRadPerSec());

        DifferentialDriveKinematics.BodyVelocity achieved = kinematics.computeBodyVelocity(commanded, c);
        setPose(kinematics.integrate(getPose(), achieved.v(), achieved.omega(), dt));
        return achieved;
    }

    // --- Property accessors, mirroring DHParameterModel's getter/property pattern ---

    public double getWheelRadius() { return wheelRadius.get(); }
    public DoubleProperty wheelRadiusProperty() { return wheelRadius; }

    public double getTrackWidth() { return trackWidth.get(); }
    public DoubleProperty trackWidthProperty() { return trackWidth; }

    public double getMaxWheelSpeedRadPerSec() { return maxWheelSpeedRadPerSec.get(); }
    public DoubleProperty maxWheelSpeedRadPerSecProperty() { return maxWheelSpeedRadPerSec; }

    public double getX() { return x.get(); }
    public DoubleProperty xProperty() { return x; }

    public double getY() { return y.get(); }
    public DoubleProperty yProperty() { return y; }

    public double getThetaDeg() { return thetaDeg.get(); }
    public DoubleProperty thetaDegProperty() { return thetaDeg; }

    public double getLeftWheelSpeed() { return leftWheelSpeed.get(); }
    public DoubleProperty leftWheelSpeedProperty() { return leftWheelSpeed; }

    public double getRightWheelSpeed() { return rightWheelSpeed.get(); }
    public DoubleProperty rightWheelSpeedProperty() { return rightWheelSpeed; }
}