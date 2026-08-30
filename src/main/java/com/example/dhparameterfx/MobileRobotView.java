package com.example.dhparameterfx;

import javafx.scene.Group;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;

/**
 * Renders a single differential-drive robot: a chassis box, two wheel
 * cylinders spaced by track width, and a forward-heading indicator.
 * <p>
 * This is the mobile-robot analogue of the arm's {@code AxisGroup} — a
 * self-contained node whose geometry is rebuilt when chassis parameters
 * change, and whose transform is updated every frame from a {@link Pose2D}.
 * It knows nothing about {@link DifferentialDriveKinematics} or
 * {@link MobileRobotModel} — it only draws whatever pose it's given, the
 * same separation of concerns as the arm's joint-drawing code in
 * {@code kinematic3DApp.updateRobot3D}.
 * <p>
 * Coordinate convention: matches the rest of the scene (see
 * {@code kinematic3DApp}'s floor plane and {@code robotGroup} base
 * rotations) — ground is the XY plane, Z is up, and heading rotates about
 * Z. Place this node's parent under the same base-rotated group the arm
 * uses if you want a shared world; for the standalone test app it is used
 * directly under a similarly-rotated root.
 */
public class MobileRobotView {

    private final Group root = new Group();
    private final Box chassis = new Box();
    private final Cylinder leftWheel = new Cylinder();
    private final Cylinder rightWheel = new Cylinder();
    private final Cylinder headingIndicator = new Cylinder();

    private static final double WHEEL_WIDTH = 0.4;
    private static final double CHASSIS_LENGTH = 2.2;
    private static final double CHASSIS_HEIGHT = 0.5;

    public MobileRobotView() {
        chassis.setMaterial(new PhongMaterial(Color.web("#61afef")));
        leftWheel.setMaterial(new PhongMaterial(Color.web("#2c313a")));
        rightWheel.setMaterial(new PhongMaterial(Color.web("#2c313a")));
        headingIndicator.setMaterial(new PhongMaterial(Color.web("#e5c07b")));

        // Wheels rotate about their axle, which for a robot facing +X runs
        // along +Y (left/right) — a JavaFX Cylinder's height axis is Y by
        // default, so no extra rotation is needed here, only translation.
        // The heading indicator, in contrast, must be laid flat along +X.
        headingIndicator.getTransforms().add(new Rotate(90, Rotate.Z_AXIS));

        root.getChildren().addAll(chassis, leftWheel, rightWheel, headingIndicator);
        updateGeometry(1.0, 3.0);
    }

    public Group getNode() {
        return root;
    }

    /**
     * Rebuilds wheel/chassis dimensions from chassis geometry. Call this
     * whenever wheelRadius or trackWidth changes (e.g. from a control-panel
     * slider), not every frame — geometry is otherwise static between
     * parameter edits.
     */
    public void updateGeometry(double wheelRadius, double trackWidth) {
        leftWheel.setRadius(wheelRadius);
        leftWheel.setHeight(WHEEL_WIDTH);
        rightWheel.setRadius(wheelRadius);
        rightWheel.setHeight(WHEEL_WIDTH);

        leftWheel.setTranslateY(trackWidth / 2.0);
        leftWheel.setTranslateZ(wheelRadius);
        rightWheel.setTranslateY(-trackWidth / 2.0);
        rightWheel.setTranslateZ(wheelRadius);

        chassis.setWidth(trackWidth + 0.8);
        chassis.setHeight(CHASSIS_HEIGHT);
        chassis.setDepth(CHASSIS_LENGTH);
        chassis.setTranslateZ(wheelRadius + CHASSIS_HEIGHT / 2.0);

        headingIndicator.setRadius(0.12);
        headingIndicator.setHeight(CHASSIS_LENGTH * 0.6);
        headingIndicator.setTranslateX(CHASSIS_LENGTH * 0.55);
        headingIndicator.setTranslateZ(wheelRadius + CHASSIS_HEIGHT / 2.0);
    }

    /** Positions/orients the whole robot node from a planar pose. */
    public void updatePose(Pose2D pose) {
        root.getTransforms().setAll(
                new Translate(pose.x(), pose.y(), 0),
                new Rotate(pose.thetaDeg(), Rotate.Z_AXIS)
        );
    }
}