package com.example.dhparameterfx;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;

/**
 * JavaFX-property-backed Ackermann (car-like) robot state: chassis geometry,
 * current pose, and current steering angle / per-wheel steering geometry.
 * This is the Ackermann counterpart to {@link MobileRobotModel} — same role
 * (observable data holder for control-panel binding, math delegated to the
 * pure {@link AckermannKinematics} engine), implementing
 * {@link PlanarRobotModel} so {@code MobileRobotTestApp} can drive either
 * drive type through one interface reference rather than branching
 * everywhere on drive type.
 * <p>
 * {@code wheelRadius} here is a rendering-only parameter — unlike
 * differential drive, wheel radius doesn't appear anywhere in the Ackermann
 * bicycle-model kinematics ({@link AckermannKinematics.Chassis} has no such
 * field), it only affects how tall the wheel cylinders are drawn.
 */
public class AckermannRobotModel implements PlanarRobotModel {

    private final AckermannKinematics kinematics = new AckermannKinematics();

    // Chassis geometry (kinematic)
    private final DoubleProperty wheelbase = new SimpleDoubleProperty(6.0);
    private final DoubleProperty trackWidth = new SimpleDoubleProperty(3.5);
    private final DoubleProperty maxSteeringAngleDeg = new SimpleDoubleProperty(35.0);
    private final DoubleProperty maxSpeed = new SimpleDoubleProperty(10.0);

    // Rendering-only
    private final DoubleProperty wheelRadius = new SimpleDoubleProperty(1.0);

    // Pose
    private final DoubleProperty x = new SimpleDoubleProperty(0.0);
    private final DoubleProperty y = new SimpleDoubleProperty(0.0);
    private final DoubleProperty thetaDeg = new SimpleDoubleProperty(0.0);

    // Last commanded/achieved steering, exposed for HUD/visualization
    private final DoubleProperty steeringAngleDeg = new SimpleDoubleProperty(0.0);
    private final DoubleProperty leftFrontWheelAngleDeg = new SimpleDoubleProperty(0.0);
    private final DoubleProperty rightFrontWheelAngleDeg = new SimpleDoubleProperty(0.0);

    public AckermannRobotModel() {
    }

    public AckermannRobotModel(double wheelbase, double trackWidth, double maxSteeringAngleDeg, double maxSpeed) {
        this.wheelbase.set(wheelbase);
        this.trackWidth.set(trackWidth);
        this.maxSteeringAngleDeg.set(maxSteeringAngleDeg);
        this.maxSpeed.set(maxSpeed);
    }

    private AckermannKinematics.Chassis chassis() {
        return new AckermannKinematics.Chassis(
                wheelbase.get(), trackWidth.get(), Math.toRadians(maxSteeringAngleDeg.get()), maxSpeed.get());
    }

    @Override
    public Pose2D getPose() {
        return new Pose2D(x.get(), y.get(), thetaDeg.get());
    }

    @Override
    public void setPose(Pose2D pose) {
        x.set(pose.x());
        y.set(pose.y());
        thetaDeg.set(pose.thetaDeg());
    }

    /**
     * Advances the simulation by dt seconds toward the requested body
     * velocity. Unlike {@link MobileRobotModel#step}, clamping here is not
     * just a hardware-limit cap: it also passes through the real v=0
     * pivot-turn singularity ({@link AckermannKinematics#computeSteeringAngle}
     * returns 0 in that case, meaning the requested turn is silently
     * unachievable, not merely capped). The steering angle and per-wheel
     * Ackermann geometry actually used are exposed via
     * {@link #getSteeringAngleDeg()}/{@link #getLeftFrontWheelAngleDeg()}/
     * {@link #getRightFrontWheelAngleDeg()} for the HUD and visualization.
     */
    @Override
    public void step(double requestedV, double requestedOmega, double dt) {
        AckermannKinematics.Chassis c = chassis();

        double clampedV = Math.max(-c.maxSpeed(), Math.min(c.maxSpeed(), requestedV));
        double delta = kinematics.computeSteeringAngle(clampedV, requestedOmega, c);
        steeringAngleDeg.set(Math.toDegrees(delta));

        AckermannKinematics.FrontWheelAngles wheelAngles = kinematics.computeAckermannWheelAngles(delta, c);
        leftFrontWheelAngleDeg.set(Math.toDegrees(wheelAngles.leftRad()));
        rightFrontWheelAngleDeg.set(Math.toDegrees(wheelAngles.rightRad()));

        AckermannKinematics.BodyVelocity achieved = kinematics.computeBodyVelocity(clampedV, delta, c);
        setPose(kinematics.integrate(getPose(), achieved.v(), achieved.omega(), dt));
    }

    // --- Property accessors ---

    public double getWheelbase() { return wheelbase.get(); }
    public DoubleProperty wheelbaseProperty() { return wheelbase; }

    public double getTrackWidth() { return trackWidth.get(); }
    public DoubleProperty trackWidthProperty() { return trackWidth; }

    public double getMaxSteeringAngleDeg() { return maxSteeringAngleDeg.get(); }
    public DoubleProperty maxSteeringAngleDegProperty() { return maxSteeringAngleDeg; }

    public double getMaxSpeed() { return maxSpeed.get(); }
    public DoubleProperty maxSpeedProperty() { return maxSpeed; }

    public double getWheelRadius() { return wheelRadius.get(); }
    public DoubleProperty wheelRadiusProperty() { return wheelRadius; }

    public double getX() { return x.get(); }
    public DoubleProperty xProperty() { return x; }

    public double getY() { return y.get(); }
    public DoubleProperty yProperty() { return y; }

    public double getThetaDeg() { return thetaDeg.get(); }
    public DoubleProperty thetaDegProperty() { return thetaDeg; }

    public double getSteeringAngleDeg() { return steeringAngleDeg.get(); }
    public DoubleProperty steeringAngleDegProperty() { return steeringAngleDeg; }

    public double getLeftFrontWheelAngleDeg() { return leftFrontWheelAngleDeg.get(); }
    public DoubleProperty leftFrontWheelAngleDegProperty() { return leftFrontWheelAngleDeg; }

    public double getRightFrontWheelAngleDeg() { return rightFrontWheelAngleDeg.get(); }
    public DoubleProperty rightFrontWheelAngleDegProperty() { return rightFrontWheelAngleDeg; }
}