package com.example.dhparameterfx;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.beans.property.DoubleProperty;
import javafx.geometry.Insets;
import javafx.geometry.Orientation;
import javafx.geometry.Point3D;
import javafx.geometry.Pos;
import javafx.scene.*;
import javafx.scene.control.*;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.CullFace;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.DrawMode;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import javafx.stage.FileChooser;
import javafx.stage.Stage;

import java.io.File;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

public class kinematic3DApp extends Application {

    private final Group world = new Group();
    private final Group robotGroup = new Group();
    private final ForwardKinematicsEngine fkEngine = new ForwardKinematicsEngine();
    private final IKSolver ikSolver = new IKSolver();

    private final List<DHParameterModel> dhModels = new ArrayList<>();
    private VBox controlsContainer;
    private MatrixDisplayHUD hud;

    private int selectedJointIndex = 0;

    // IK & Trajectory state
    private final Sphere targetSphere = new Sphere(1.2);
    private final double[] targetPos = new double[]{10.0, 5.0, 5.0};
    private AnimationTimer playbackTimer;

    private boolean useCartesianTrajectory = true;

    // --- Persistent scene-graph containers ---
    // jointsGroup holds the per-joint axis/link/highlight nodes. It is fully torn down and
    // rebuilt only on STRUCTURAL changes (add/remove joint, preset load, selection change,
    // manual slider edit). During animation playback we never touch it structurally - we only
    // update the transforms of the nodes already inside it (see refreshJointTransforms()).
    private final Group jointsGroup = new Group();
    private final Group floorPlaneGroup = new Group();
    private final Group trailGroup = new Group();

    // Node caches so the 60fps animation loop can update transforms in place instead of
    // allocating new AxisGroup/Cylinder/Material objects every frame.
    private final List<AxisGroup> jointAxisNodes = new ArrayList<>();
    private final List<Cylinder> linkCylinderNodes = new ArrayList<>();
    private Node highlightBoxNode;

    // Joint type tracking. IKSolver/TrajectoryPlanner (not visible to this file) only ever
    // read/write theta, so this is tracked locally and kept in sync with dhModels at every
    // mutation site (add/remove/preset/import). true = prismatic (driven by d), false =
    // revolute (driven by theta).
    private final List<Boolean> jointIsPrismatic = new ArrayList<>();

    /** Reads whichever DH variable is this joint's driven DOF (theta for revolute, d for prismatic). */
    private double getJointVar(int i) {
        return jointIsPrismatic.get(i) ? dhModels.get(i).getD() : dhModels.get(i).getTheta();
    }

    /** Writes whichever DH variable is this joint's driven DOF (theta for revolute, d for prismatic). */
    private void setJointVar(int i, double value) {
        if (jointIsPrismatic.get(i)) {
            dhModels.get(i).dProperty().set(value);
        } else {
            dhModels.get(i).setTheta(value);
        }
    }

    // Floor / restriction-plane state
    private boolean showFloorPlane = true;
    private static final double FLOOR_EPSILON = 0.05;

    // Elementary transform breakdown panel: cached Label nodes so we only ever setText() on
    // them (called from the same hot path as the HUD, up to 60fps) rather than rebuilding
    // GridPanes every frame.
    private boolean showTransformBreakdown = false;
    private VBox transformBreakdownPanel;
    private Label breakdownStatusLabel;
    private Label selfTestResultLabel;
    private final Label[][] rzLabels = new Label[4][4];
    private final Label[][] tzLabels = new Label[4][4];
    private final Label[][] txLabels = new Label[4][4];
    private final Label[][] rxLabels = new Label[4][4];

    // Planned-path trail state
    private boolean showPlannedTrail = true;

    /**
     * Builds a visible restriction plane at Z = 0 (the "floor" the arm's joints must not dip
     * below). Lives in the same local frame as the DH-computed joint positions, so it lines
     * up exactly with the Z >= 0 constraint enforced elsewhere. Sized as a square of the given
     * extent, with a light reference grid every {@code gridSpacing} units.
     */
    private Node buildFloorPlane(double extent, double gridSpacing) {
        Group plane = new Group();

        Box fill = new Box(extent, extent, 0.05);
        PhongMaterial fillMat = new PhongMaterial();
        fillMat.setDiffuseColor(Color.web("#61afef", 0.12));
        fillMat.setSpecularColor(Color.TRANSPARENT);
        fill.setMaterial(fillMat);
        fill.setDrawMode(DrawMode.FILL);
        fill.setCullFace(CullFace.NONE);
        fill.setTranslateZ(0);
        plane.getChildren().add(fill);

        PhongMaterial lineMat = new PhongMaterial(Color.web("#61afef", 0.35));
        double lineThickness = 0.06;
        double half = extent / 2.0;
        for (double x = -half; x <= half + 1e-6; x += gridSpacing) {
            Box line = new Box(lineThickness, extent, 0.06);
            line.setMaterial(lineMat);
            line.setTranslateX(x);
            line.setTranslateZ(0.02);
            plane.getChildren().add(line);
        }
        for (double y = -half; y <= half + 1e-6; y += gridSpacing) {
            Box line = new Box(extent, lineThickness, 0.06);
            line.setMaterial(lineMat);
            line.setTranslateY(y);
            line.setTranslateZ(0.02);
            plane.getChildren().add(line);
        }

        return plane;
    }

    private Node createSelectionHighlightBox(double size) {
        Box box = new Box(size, size, size);
        box.setDrawMode(DrawMode.LINE);
        box.setCullFace(CullFace.NONE);
        box.setMaterial(new PhongMaterial(Color.web("#61afef")));
        return box;
    }

    @Override
    public void start(Stage primaryStage) {

        SubScene subScene = new SubScene(world, 800, 700, true, SceneAntialiasing.BALANCED);
        subScene.setFill(Color.web("#1e1e24"));

        OrbitCamera cameraRig = new OrbitCamera();
        subScene.setCamera(cameraRig.getCamera());

        world.getChildren().add(cameraRig.getRootNode());
        robotGroup.getTransforms().add(new Rotate(-270, Rotate.X_AXIS));
        robotGroup.getTransforms().add(new Rotate(270, Rotate.Z_AXIS));
        world.getChildren().add(robotGroup);

        // Target Sphere
        targetSphere.setMaterial(new PhongMaterial(Color.web("#e06c75")));
        robotGroup.getChildren().add(targetSphere);
        updateTargetSpherePosition();

        // Persistent containers: added ONCE. jointsGroup contents get rebuilt on structural
        // changes; floorPlaneGroup/trailGroup are toggled via visibility, never torn down.
        floorPlaneGroup.getChildren().add(buildFloorPlane(30.0, 5.0));
        floorPlaneGroup.setVisible(showFloorPlane);
        floorPlaneGroup.setMouseTransparent(true);

        trailGroup.setVisible(showPlannedTrail);
        trailGroup.setMouseTransparent(true);

        robotGroup.getChildren().addAll(floorPlaneGroup, trailGroup, jointsGroup);

        // Lighting
        AmbientLight ambient = new AmbientLight(Color.color(0.4, 0.4, 0.4));
        PointLight pointLight = new PointLight(Color.WHITE);
        pointLight.setTranslateX(-20);
        pointLight.setTranslateY(-40);
        pointLight.setTranslateZ(-50);
        world.getChildren().addAll(ambient, pointLight);

        // Default 3-Joint Robot
        dhModels.add(new DHParameterModel(0.0, 90.0, 5.0, 30.0));
        dhModels.add(new DHParameterModel(10.0, 0.0, 0.0, 40.0));
        dhModels.add(new DHParameterModel(8.0, 0.0, 0.0, -25.0));
        jointIsPrismatic.addAll(List.of(false, false, false));

        // Create HUD Overlay
        hud = new MatrixDisplayHUD();

        // Elementary transform breakdown overlay, built once (labels updated via setText
        // afterward, never rebuilt per-frame).
        transformBreakdownPanel = buildTransformBreakdownPanel();
        transformBreakdownPanel.setVisible(showTransformBreakdown);
        transformBreakdownPanel.setMouseTransparent(true);

        // Layer SubScene and HUD using StackPane
        StackPane viewportPane = new StackPane();
        viewportPane.getChildren().addAll(subScene, hud, transformBreakdownPanel);
        StackPane.setAlignment(hud, Pos.TOP_LEFT);
        StackPane.setMargin(hud, new Insets(15));
        StackPane.setAlignment(transformBreakdownPanel, Pos.BOTTOM_LEFT);
        StackPane.setMargin(transformBreakdownPanel, new Insets(15));

        BorderPane root = new BorderPane();
        root.setCenter(viewportPane);

        subScene.widthProperty().bind(viewportPane.widthProperty());
        subScene.heightProperty().bind(viewportPane.heightProperty());

        VBox sidePanel = createControlPanel(primaryStage);
        root.setRight(sidePanel);

        Scene scene = new Scene(root, 1150, 750);

        subScene.setOnMouseClicked(event -> {
            Node picked = event.getPickResult().getIntersectedNode();
            Node current = picked;
            while (current != null && !(current instanceof AxisGroup) && current != robotGroup) {
                current = current.getParent();
            }

            if (current instanceof AxisGroup axis) {
                Object tag = axis.getUserData();
                if (tag instanceof Integer jointIdx) {
                    selectedJointIndex = jointIdx;
                    rebuildUIControls();
                    updateRobot3D();
                }
            }
        });

        cameraRig.registerMouseEvents(scene);

        rebuildUIControls();
        updateRobot3D();

        primaryStage.setTitle("JavaFX DH Parameter & Kinematics Workspace");
        primaryStage.setScene(scene);
        primaryStage.show();
    }

    /**
     * Runs the SAME per-joint verification the breakdown panel does (actual step, derived from
     * the engine's own cumulative output, vs the independently-computed Rz*Tz*Tx*Rx product),
     * but across every joint in the chain at once, so the whole thing can be checked in one
     * action instead of clicking through each joint individually.
     */
    private void runFullChainSelfTest() {
        List<Matrix4x4> transforms = computeCurrentTransforms();
        if (transforms.isEmpty()) {
            selfTestResultLabel.setText("No joints to test.");
            selfTestResultLabel.setStyle("-fx-text-fill: #abb2bf; -fx-font-size: 11px;");
            return;
        }

        double worstDiff = 0;
        int worstJoint = -1;

        for (int i = 0; i < dhModels.size(); i++) {
            DHParameterModel model = dhModels.get(i);

            double[][] rz = rotZMatrix(model.getTheta());
            double[][] tz = transZMatrix(model.getD());
            double[][] tx = transXMatrix(model.getA());
            double[][] rx = rotXMatrix(model.getAlpha());
            double[][] candidateStep = matMul4(matMul4(rz, tz), matMul4(tx, rx));

            double[][] prevCumulative = (i == 0) ? identity4() : toArray(transforms.get(i - 1));
            double[][] currCumulative = toArray(transforms.get(i));
            double[][] actualStep = matMul4(invertRigid(prevCumulative), currCumulative);

            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 4; c++) {
                    double diff = Math.abs(actualStep[r][c] - candidateStep[r][c]);
                    if (diff > worstDiff) {
                        worstDiff = diff;
                        worstJoint = i;
                    }
                }
            }
        }

        if (worstDiff < 1e-3) {
            selfTestResultLabel.setText(String.format(
                    "\u2713 All %d joint(s) match the standard Rz\u00B7Tz\u00B7Tx\u00B7Rx convention (max discrepancy %.5f)",
                    dhModels.size(), worstDiff));
            selfTestResultLabel.setStyle("-fx-text-fill: #98c379; -fx-font-size: 11px; -fx-font-weight: bold;");
        } else {
            selfTestResultLabel.setText(String.format(
                    "\u26A0 Joint %d diverges from the standard convention by %.3f - the other joint(s) check out",
                    worstJoint + 1, worstDiff));
            selfTestResultLabel.setStyle("-fx-text-fill: #e5c07b; -fx-font-size: 11px; -fx-font-weight: bold;");
        }
    }

    private void updateTargetSpherePosition() {
        targetSphere.setTranslateX(targetPos[0]);
        targetSphere.setTranslateY(targetPos[1]);
        targetSphere.setTranslateZ(targetPos[2]);
    }

    /** Builds one small labeled 4x4 grid of cached Label cells for a matrix card. */
    private VBox buildMatrixCard(String title, Label[][] labelRefs) {
        Label header = new Label(title);
        header.setStyle("-fx-text-fill: #61afef; -fx-font-weight: bold; -fx-font-size: 10px;");

        GridPane grid = new GridPane();
        grid.setHgap(5);
        grid.setVgap(2);
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                Label cell = new Label("0.00");
                cell.setStyle("-fx-text-fill: #d0d0d0; -fx-font-family: monospace; -fx-font-size: 10px;");
                cell.setMinWidth(42);
                cell.setAlignment(Pos.CENTER_RIGHT);
                grid.add(cell, c, r);
                labelRefs[r][c] = cell;
            }
        }

        VBox card = new VBox(3, header, grid);
        card.setStyle("-fx-background-color: rgba(30,30,36,0.9); -fx-padding: 6; -fx-background-radius: 4;");
        return card;
    }

    /** Builds the full elementary-transform-breakdown overlay panel, once. */
    private VBox buildTransformBreakdownPanel() {
        VBox rzCard = buildMatrixCard("Rot_z(\u03B8)", rzLabels);
        VBox tzCard = buildMatrixCard("Trans_z(d)", tzLabels);
        VBox txCard = buildMatrixCard("Trans_x(a)", txLabels);
        VBox rxCard = buildMatrixCard("Rot_x(\u03B1)", rxLabels);

        HBox matricesRow = new HBox(8, rzCard, tzCard, txCard, rxCard);

        Label title = new Label("Elementary Transforms - Selected Joint");
        title.setStyle("-fx-text-fill: white; -fx-font-weight: bold; -fx-font-size: 11px;");

        breakdownStatusLabel = new Label();
        breakdownStatusLabel.setWrapText(true);
        breakdownStatusLabel.setMaxWidth(400);
        breakdownStatusLabel.setStyle("-fx-font-size: 10px; -fx-font-weight: bold;");

        VBox panel = new VBox(6, title, matricesRow, breakdownStatusLabel);
        panel.setStyle("-fx-background-color: rgba(20,20,24,0.8); -fx-padding: 10; -fx-background-radius: 6;");
        return panel;
    }

    // --- Plain 4x4 matrix math, independent of Matrix4x4/ForwardKinematicsEngine internals.
    // Used only to build the 4 candidate elementary transforms and to verify them against the
    // engine's own actual output - see updateTransformBreakdown() below. ---

    private double[][] matMul4(double[][] A, double[][] B) {
        double[][] R = new double[4][4];
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                double sum = 0;
                for (int k = 0; k < 4; k++) sum += A[r][k] * B[k][c];
                R[r][c] = sum;
            }
        }
        return R;
    }

    private double[][] rotZMatrix(double thetaDeg) {
        double t = Math.toRadians(thetaDeg);
        double c = Math.cos(t), s = Math.sin(t);
        return new double[][]{{c, -s, 0, 0}, {s, c, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    }

    private double[][] rotXMatrix(double alphaDeg) {
        double a = Math.toRadians(alphaDeg);
        double c = Math.cos(a), s = Math.sin(a);
        return new double[][]{{1, 0, 0, 0}, {0, c, -s, 0}, {0, s, c, 0}, {0, 0, 0, 1}};
    }

    private double[][] transZMatrix(double d) {
        return new double[][]{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, d}, {0, 0, 0, 1}};
    }

    private double[][] transXMatrix(double a) {
        return new double[][]{{1, 0, 0, a}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    }

    private double[][] identity4() {
        return new double[][]{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    }

    /** Extracts a plain 4x4 array from a Matrix4x4. Row 3 is hardcoded to [0,0,0,1] rather than
     * queried, since only rows 0-2 are used anywhere else in this file (get(3,c) is untested). */
    private double[][] toArray(Matrix4x4 m) {
        double[][] out = new double[4][4];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 4; c++) {
                out[r][c] = m.get(r, c);
            }
        }
        out[3] = new double[]{0, 0, 0, 1};
        return out;
    }

    /** Inverse of a rigid (rotation + translation) homogeneous transform: [R^T, -R^T p; 0 0 0 1]. */
    private double[][] invertRigid(double[][] T) {
        double[][] inv = new double[4][4];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                inv[r][c] = T[c][r];
            }
        }
        for (int r = 0; r < 3; r++) {
            double sum = 0;
            for (int k = 0; k < 3; k++) sum += inv[r][k] * T[k][3];
            inv[r][3] = -sum;
        }
        inv[3] = new double[]{0, 0, 0, 1};
        return inv;
    }

    private void setGridValues(Label[][] labels, double[][] values) {
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                labels[r][c].setText(String.format("%.2f", values[r][c]));
            }
        }
    }

    /**
     * Updates the elementary-transform breakdown for the currently selected joint, and
     * self-verifies it: rather than assuming this engine uses the standard Rz*Tz*Tx*Rx DH
     * convention, the ACTUAL step transform is derived from the engine's own cumulative
     * output (by inverting the previous joint's transform into the current one), and compared
     * against the candidate product. If they don't match, the breakdown is flagged as such
     * instead of silently presenting unverified numbers.
     */
    private void updateTransformBreakdown(List<Matrix4x4> transforms) {
        if (!showTransformBreakdown || transforms.isEmpty()) return;

        int idx = Math.max(0, Math.min(selectedJointIndex, transforms.size() - 1));
        DHParameterModel model = dhModels.get(idx);

        double[][] rz = rotZMatrix(model.getTheta());
        double[][] tz = transZMatrix(model.getD());
        double[][] tx = transXMatrix(model.getA());
        double[][] rx = rotXMatrix(model.getAlpha());

        setGridValues(rzLabels, rz);
        setGridValues(tzLabels, tz);
        setGridValues(txLabels, tx);
        setGridValues(rxLabels, rx);

        double[][] prevCumulative = (idx == 0) ? identity4() : toArray(transforms.get(idx - 1));
        double[][] currCumulative = toArray(transforms.get(idx));
        double[][] actualStep = matMul4(invertRigid(prevCumulative), currCumulative);

        double[][] candidateStep = matMul4(matMul4(rz, tz), matMul4(tx, rx));

        double maxDiff = 0;
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 4; c++) {
                maxDiff = Math.max(maxDiff, Math.abs(actualStep[r][c] - candidateStep[r][c]));
            }
        }

        if (maxDiff < 1e-3) {
            breakdownStatusLabel.setText("\u2713 Product matches the engine's actual step transform (standard Rz\u00B7Tz\u00B7Tx\u00B7Rx convention confirmed)");
            breakdownStatusLabel.setStyle("-fx-text-fill: #98c379; -fx-font-size: 10px; -fx-font-weight: bold;");
        } else {
            breakdownStatusLabel.setText(String.format(
                    "\u26A0 Product differs from the engine's actual step transform by up to %.3f - this engine may use a different DH convention than Rz\u00B7Tz\u00B7Tx\u00B7Rx",
                    maxDiff));
            breakdownStatusLabel.setStyle("-fx-text-fill: #e5c07b; -fx-font-size: 10px; -fx-font-weight: bold;");
        }
    }

    /** Computes the cumulative FK transforms for the DH chain's current pose. */
    private List<Matrix4x4> computeCurrentTransforms() {
        List<DHParameter> dhParams = new ArrayList<>();
        for (DHParameterModel model : dhModels) {
            dhParams.add(model.toDHParameter());
        }
        return fkEngine.computeCumulativeTransforms(dhParams);
    }

    /** True if any joint in the chain has dipped below the Z = 0 floor plane. */
    private boolean violatesFloor(List<Matrix4x4> transforms) {
        for (Matrix4x4 m : transforms) {
            if (m.getPosition()[2] < -FLOOR_EPSILON) return true;
        }
        return false;
    }

    /**
     * Updates the HUD with the transform of the currently SELECTED joint (not always the
     * end-effector), so the HUD actually functions as a DH-parameter checker: pick a joint,
     * see the matrix that joint's (a, alpha, d, theta) row produces. Falls back to a valid
     * index if the selection is stale (e.g. right after a joint was deleted).
     */
    private void updateHudDisplay(List<Matrix4x4> transforms) {
        if (transforms.isEmpty() || hud == null) return;
        int idx = Math.max(0, Math.min(selectedJointIndex, transforms.size() - 1));
        hud.update(transforms.get(idx));
        updateTransformBreakdown(transforms);
    }

    /**
     * Draws a visible 3D trail through the workspace tracing the path the end-effector is
     * about to take, BEFORE the arm moves. For Cartesian trajectories this is a straight line
     * from the current end-effector position to the target. For joint-space trajectories the
     * path is generally curved in Cartesian space, so we sample the interpolated joint
     * trajectory at several points and connect the resulting end-effector positions.
     */
    private void buildPlannedTrail(double[] qStart, double[] qEnd, double[] startPos) {
        trailGroup.getChildren().clear();
        Color trailColor = Color.web("#98c379", 0.85);

        List<Point3D> samples = new ArrayList<>();

        if (useCartesianTrajectory) {
            samples.add(new Point3D(startPos[0], startPos[1], startPos[2]));
            samples.add(new Point3D(targetPos[0], targetPos[1], Math.max(0.0, targetPos[2])));
        } else {
            int sampleCount = 24;
            for (int s = 0; s <= sampleCount; s++) {
                double t = (double) s / sampleCount;
                double[] q = TrajectoryPlanner.interpolateCubic(qStart, qEnd, t);
                for (int i = 0; i < dhModels.size(); i++) {
                    setJointVar(i, q[i]);
                }
                List<Matrix4x4> transforms = computeCurrentTransforms();
                double[] p = transforms.get(transforms.size() - 1).getPosition();
                samples.add(new Point3D(p[0], p[1], p[2]));
            }
            // Restore the chain to its starting pose - the caller resets to qStart right after
            // this returns, but we leave it consistent either way.
            for (int i = 0; i < dhModels.size(); i++) {
                setJointVar(i, qStart[i]);
            }
        }

        for (int i = 1; i < samples.size(); i++) {
            Node segment = createLinkCylinder(samples.get(i - 1), samples.get(i), 0.15, trailColor);
            trailGroup.getChildren().add(segment);
        }

        // A small marker sphere at the destination makes the endpoint of the plan unambiguous.
        Sphere endMarker = new Sphere(0.4);
        endMarker.setMaterial(new PhongMaterial(trailColor));
        Point3D lastPoint = samples.get(samples.size() - 1);
        endMarker.setTranslateX(lastPoint.getX());
        endMarker.setTranslateY(lastPoint.getY());
        endMarker.setTranslateZ(lastPoint.getZ());
        trailGroup.getChildren().add(endMarker);
    }

    private void runIKAndAnimate() {
        // Check basic workspace reachability
        if (!ikSolver.isTargetReachable(dhModels, targetPos)) {
            showWarningDialog("Unreachable Target",
                    String.format("The target position (%.1f, %.1f, %.1f) is beyond the robot's kinematic reach.",
                            targetPos[0], targetPos[1], targetPos[2]));
            return;
        }

        // 1. Capture starting configuration. qStart[i] holds theta for revolute joints, d for
        // prismatic ones - the "driven DOF" per joint.
        double[] qStart = new double[dhModels.size()];
        for (int i = 0; i < dhModels.size(); i++) {
            qStart[i] = getJointVar(i);
        }

        // 2. Solve IK (350 iterations, 0.1 tolerance). IKSolver now takes jointIsPrismatic
        // directly, so it builds a proper Jacobian column (and applies updates) against d for
        // prismatic joints and theta for revolute ones - no post-hoc undo or separate
        // coordinate-descent pass needed anymore.
        boolean solved = ikSolver.solve(dhModels, targetPos, 350, 0.1, jointIsPrismatic);

        if (!solved) {
            showWarningDialog("Target Unreachable",
                    "The manipulator reached its limit towards the target, but could not match the exact position due to joint constraints.");
            // Reset to start position and abort
            for (int i = 0; i < dhModels.size(); i++) {
                setJointVar(i, qStart[i]);
            }
            return;
        }

        // 3. Capture end configuration after the solver finishes successfully
        double[] qEnd = new double[dhModels.size()];
        for (int i = 0; i < dhModels.size(); i++) {
            qEnd[i] = getJointVar(i);
        }

        // 4. Reset the models back to the start so we can animate from the beginning
        for (int i = 0; i < dhModels.size(); i++) {
            setJointVar(i, qStart[i]);
        }

        // 5. Compute transforms at the starting pose to define startPos for the straight line
        List<Matrix4x4> startTransforms = computeCurrentTransforms();
        double[] startPos = startTransforms.get(startTransforms.size() - 1).getPosition();

        // 6. Build the visible planned-path trail BEFORE any motion happens, so the user can
        // see exactly where the arm is about to go.
        buildPlannedTrail(qStart, qEnd, startPos);

        // dhModels may have been left mid-sample by buildPlannedTrail's joint-space preview;
        // guarantee we're sitting at the start pose before animating.
        for (int i = 0; i < dhModels.size(); i++) {
            setJointVar(i, qStart[i]);
        }

        // 7. Stop any existing animation
        if (playbackTimer != null) playbackTimer.stop();

        // 8. Start the new animation timer
        final long startTime = System.nanoTime();
        final double durationNs = 1_500_000_000.0; // 1.5 seconds

        // Tracks the last configuration that did NOT violate the floor, so a frame that would
        // dip a joint below Z = 0 can be rejected without freezing the whole animation.
        final double[] lastGoodQ = qStart.clone();

        playbackTimer = new AnimationTimer() {
            @Override
            public void handle(long now) {
                double elapsed = now - startTime;
                double tNorm = Math.max(0.0, Math.min(1.0, elapsed / durationNs));

                // Smooth cubic easing s(t) = 3t^2 - 2t^3
                double s = 3 * tNorm * tNorm - 2 * tNorm * tNorm * tNorm;

                if (useCartesianTrajectory) {
                    // CARTESIAN SPACE: Interpolate end-effector in a straight line
                    double[] currentTarget = new double[]{
                            startPos[0] + s * (targetPos[0] - startPos[0]),
                            startPos[1] + s * (targetPos[1] - startPos[1]),
                            Math.max(0.0, startPos[2] + s * (targetPos[2] - startPos[2])) // Floor constraint
                    };

                    // Run a quick 5-iteration IK to track the line on this frame. Now that
                    // IKSolver understands jointIsPrismatic, this correctly drives d for
                    // prismatic joints and theta for revolute ones, frame by frame - no manual
                    // post-processing needed.
                    ikSolver.solve(dhModels, currentTarget, 5, 0.1, jointIsPrismatic);
                } else {
                    // JOINT SPACE: Interpolate the driven variable directly (theta for
                    // revolute joints, d for prismatic ones)
                    double[] currentQ = TrajectoryPlanner.interpolateCubic(qStart, qEnd, tNorm);
                    for (int i = 0; i < dhModels.size(); i++) {
                        setJointVar(i, currentQ[i]);
                    }
                }

                // Floor enforcement: no joint (not just the end-effector) may dip below Z = 0.
                // If this frame's solution violates that, roll back to the last good pose
                // instead of applying it - the arm simply pauses at the floor boundary rather
                // than clipping through it.
                List<Matrix4x4> frameTransforms = computeCurrentTransforms();
                if (violatesFloor(frameTransforms)) {
                    for (int i = 0; i < dhModels.size(); i++) {
                        setJointVar(i, lastGoodQ[i]);
                    }
                } else {
                    for (int i = 0; i < dhModels.size(); i++) {
                        lastGoodQ[i] = getJointVar(i);
                    }
                }

                // Fast path: update existing node transforms only. No geometry is allocated
                // here, which is what makes 60fps playback smooth (including the very first
                // move) instead of rebuilding the whole robot's mesh every frame.
                refreshJointTransforms();

                if (tNorm >= 1.0) {
                    stop();
                }
            }
        };
        playbackTimer.start();
    }


    private void showWarningDialog(String title, String message) {
        Alert alert = new Alert(Alert.AlertType.WARNING);
        alert.setTitle(title);
        alert.setHeaderText(null);
        alert.setContentText(message);
        alert.showAndWait();
    }

    private VBox createControlPanel(Stage primaryStage) {
        VBox panel = new VBox(10);
        panel.setPrefWidth(340);
        panel.setPadding(new Insets(15));
        panel.setStyle("-fx-background-color: #2b2b36; -fx-text-fill: white;");

        Label header = new Label("Kinematic Chain Setup");
        header.setStyle("-fx-font-size: 16px; -fx-font-weight: bold; -fx-text-fill: white;");

        Button addBtn = new Button("+ Add Joint");
        addBtn.setMaxWidth(Double.MAX_VALUE);
        addBtn.setStyle("-fx-background-color: #98c379; -fx-text-fill: #1e1e24; -fx-font-weight: bold;");
        addBtn.setOnAction(e -> {
            dhModels.add(new DHParameterModel(5.0, 0.0, 0.0, 0.0));
            jointIsPrismatic.add(false);
            selectedJointIndex = dhModels.size() - 1;
            rebuildUIControls();
            updateRobot3D();
        });

        // Presets Toolbar
        Label presetsLabel = new Label("Presets:");
        presetsLabel.setStyle("-fx-font-size: 11px; -fx-text-fill: #abb2bf; -fx-font-weight: bold;");

        HBox presetBar = new HBox(8);
        Button scaraBtn = new Button("SCARA");
        scaraBtn.setMaxWidth(Double.MAX_VALUE);
        HBox.setHgrow(scaraBtn, Priority.ALWAYS);
        scaraBtn.setStyle("-fx-background-color: #3b3b4d; -fx-text-fill: #61afef; -fx-border-color: #61afef; -fx-border-radius: 3;");
        scaraBtn.setOnAction(e -> loadScaraPreset());

        Button pumaBtn = new Button("PUMA 560");
        pumaBtn.setMaxWidth(Double.MAX_VALUE);
        HBox.setHgrow(pumaBtn, Priority.ALWAYS);
        pumaBtn.setStyle("-fx-background-color: #3b3b4d; -fx-text-fill: #e5c07b; -fx-border-color: #e5c07b; -fx-border-radius: 3;");
        pumaBtn.setOnAction(e -> loadPuma560Preset());
        presetBar.getChildren().addAll(scaraBtn, pumaBtn);

        // File I/O Toolbar
        Label fileLabel = new Label("File I/O:");
        fileLabel.setStyle("-fx-font-size: 11px; -fx-text-fill: #abb2bf; -fx-font-weight: bold;");

        HBox fileBar = new HBox(8);
        Button exportBtn = new Button("Export JSON");
        exportBtn.setMaxWidth(Double.MAX_VALUE);
        HBox.setHgrow(exportBtn, Priority.ALWAYS);
        exportBtn.setStyle("-fx-background-color: #3b3b4d; -fx-text-fill: #98c379; -fx-border-color: #98c379; -fx-border-radius: 3;");
        exportBtn.setOnAction(e -> exportToJson(primaryStage));

        Button importBtn = new Button("Import JSON");
        importBtn.setMaxWidth(Double.MAX_VALUE);
        HBox.setHgrow(importBtn, Priority.ALWAYS);
        importBtn.setStyle("-fx-background-color: #3b3b4d; -fx-text-fill: #c678dd; -fx-border-color: #c678dd; -fx-border-radius: 3;");
        importBtn.setOnAction(e -> importFromJson(primaryStage));
        fileBar.getChildren().addAll(exportBtn, importBtn);

        // IK Target Controls Panel
        Label ikLabel = new Label("IK Target Workspace:");
        ikLabel.setStyle("-fx-font-size: 11px; -fx-text-fill: #abb2bf; -fx-font-weight: bold;");

        VBox ikBox = new VBox(6);
        ikBox.setStyle("-fx-background-color: #21252b; -fx-padding: 10; -fx-background-radius: 5; -fx-border-color: #61afef; -fx-border-radius: 5;");

        HBox targetXRow = createTargetRow("Target X:", targetPos, 0);
        HBox targetYRow = createTargetRow("Target Y:", targetPos, 1);
        HBox targetZRow = createTargetRow("Target Z:", targetPos, 2);

        Button planMotionBtn = new Button("Move to Target (IK + Trajectory)");
        planMotionBtn.setMaxWidth(Double.MAX_VALUE);
        planMotionBtn.setStyle("-fx-background-color: #61afef; -fx-text-fill: #1e1e24; -fx-font-weight: bold;");
        planMotionBtn.setOnAction(e -> runIKAndAnimate());

        CheckBox floorPlaneToggle = new CheckBox("Show floor plane (Z ≥ 0 limit)");
        floorPlaneToggle.setSelected(showFloorPlane);
        floorPlaneToggle.setStyle("-fx-text-fill: #abb2bf; -fx-font-size: 11px;");
        floorPlaneToggle.selectedProperty().addListener((o, oldV, newV) -> {
            showFloorPlane = newV;
            floorPlaneGroup.setVisible(newV);
        });

        CheckBox trailToggle = new CheckBox("Show planned path trail");
        trailToggle.setSelected(showPlannedTrail);
        trailToggle.setStyle("-fx-text-fill: #abb2bf; -fx-font-size: 11px;");
        trailToggle.selectedProperty().addListener((o, oldV, newV) -> {
            showPlannedTrail = newV;
            trailGroup.setVisible(newV);
        });

        CheckBox breakdownToggle = new CheckBox("Show elementary transform breakdown");
        breakdownToggle.setSelected(showTransformBreakdown);
        breakdownToggle.setStyle("-fx-text-fill: #abb2bf; -fx-font-size: 11px;");
        breakdownToggle.selectedProperty().addListener((o, oldV, newV) -> {
            showTransformBreakdown = newV;
            transformBreakdownPanel.setVisible(newV);
            if (newV) {
                updateTransformBreakdown(computeCurrentTransforms());
            }
        });

        Button selfTestBtn = new Button("Run Self-Test (verify whole chain)");
        selfTestBtn.setMaxWidth(Double.MAX_VALUE);
        selfTestBtn.setStyle("-fx-background-color: #3b3b4d; -fx-text-fill: #abb2bf; -fx-font-size: 11px;");
        selfTestBtn.setOnAction(e -> runFullChainSelfTest());

        selfTestResultLabel = new Label();
        selfTestResultLabel.setWrapText(true);
        selfTestResultLabel.setMaxWidth(300);

        ikBox.getChildren().addAll(targetXRow, targetYRow, targetZRow, planMotionBtn,
                floorPlaneToggle, trailToggle, breakdownToggle, selfTestBtn, selfTestResultLabel);

        controlsContainer = new VBox(12);
        ScrollPane scrollPane = new ScrollPane(controlsContainer);
        scrollPane.setFitToWidth(true);
        scrollPane.setStyle("-fx-background: transparent; -fx-background-color: transparent;");
        VBox.setVgrow(scrollPane, Priority.ALWAYS);

        panel.getChildren().addAll(
                header, addBtn, presetsLabel, presetBar, fileLabel, fileBar, ikLabel, ikBox,
                new Separator(Orientation.HORIZONTAL), scrollPane
        );

        return panel;
    }

    private HBox createTargetRow(String label, double[] posArray, int index) {
        HBox row = new HBox(8);
        row.setAlignment(Pos.CENTER_LEFT);

        Label lbl = new Label(label);
        lbl.setPrefWidth(65);
        lbl.setStyle("-fx-text-fill: #abb2bf; -fx-font-size: 11px;");

        // Index 2 is the Z (height/floor) axis - the target ball can never be placed below
        // the floor plane, so its slider and text input floor at 0 instead of -25.
        double minVal = (index == 2) ? 0.0 : -25.0;
        posArray[index] = Math.max(minVal, posArray[index]);

        Slider slider = new Slider(minVal, 25, posArray[index]);
        HBox.setHgrow(slider, Priority.ALWAYS);

        TextField txt = new TextField(String.format("%.2f", posArray[index]));
        txt.setPrefWidth(65);
        txt.setStyle("-fx-background-color: #1e1e24; -fx-text-fill: #e06c75; -fx-font-size: 11px; -fx-border-color: #4b5263; -fx-border-radius: 3;");

        slider.valueProperty().addListener((o, oldV, newV) -> {
            if (!txt.isFocused()) {
                posArray[index] = Math.max(minVal, newV.doubleValue());
                txt.setText(String.format("%.2f", posArray[index]));
                updateTargetSpherePosition();
            }
        });

        Runnable applyTxt = () -> {
            try {
                double parsed = Math.max(minVal, ExpressionParser.parse(txt.getText()));
                posArray[index] = parsed;
                slider.setValue(parsed);
                txt.setText(String.format("%.2f", parsed));
                updateTargetSpherePosition();
            } catch (Exception ex) {
                txt.setStyle("-fx-background-color: #1e1e24; -fx-text-fill: #e06c75; -fx-font-size: 11px; -fx-border-color: #e06c75; -fx-border-radius: 3;");
            }
        };

        txt.setOnAction(e -> applyTxt.run());
        txt.focusedProperty().addListener((o, wasF, isF) -> { if (!isF) applyTxt.run(); });

        row.getChildren().addAll(lbl, slider, txt);
        return row;
    }

    private void rebuildUIControls() {
        controlsContainer.getChildren().clear();

        for (int i = 0; i < dhModels.size(); i++) {
            int index = i;
            DHParameterModel model = dhModels.get(i);

            VBox card = new VBox(8);
            boolean isSelected = (i == selectedJointIndex);
            String borderStyle = isSelected ? "-fx-border-color: #61afef; -fx-border-width: 2px; -fx-border-radius: 5;" : "";
            String bgStyle = isSelected ? "-fx-background-color: #2c313a;" : "-fx-background-color: #3b3b4d;";

            card.setStyle(bgStyle + " -fx-padding: 10; -fx-background-radius: 5; " + borderStyle);

            card.setOnMouseClicked(e -> {
                selectedJointIndex = index;
                rebuildUIControls();
                updateRobot3D();
            });

            HBox cardHeader = new HBox(10);
            cardHeader.setAlignment(Pos.CENTER_LEFT);
            Label title = new Label("Joint " + (i + 1) + (isSelected ? " (Selected)" : ""));
            title.setStyle("-fx-font-weight: bold; -fx-text-fill: " + (isSelected ? "#61afef" : "#abb2bf") + ";");

            boolean isPrismatic = jointIsPrismatic.get(i);
            CheckBox prismaticToggle = new CheckBox("Prismatic");
            prismaticToggle.setSelected(isPrismatic);
            prismaticToggle.setStyle("-fx-text-fill: #e5c07b; -fx-font-size: 10px;");
            prismaticToggle.selectedProperty().addListener((o, oldV, newV) -> {
                jointIsPrismatic.set(index, newV);
                rebuildUIControls();
                updateRobot3D();
            });

            Region spacer = new Region();
            HBox.setHgrow(spacer, Priority.ALWAYS);

            Button deleteBtn = new Button("X");
            deleteBtn.setStyle("-fx-background-color: #e06c75; -fx-text-fill: white; -fx-font-size: 10px;");
            deleteBtn.setOnAction(e -> {
                if (dhModels.size() > 1) {
                    dhModels.remove(index);
                    jointIsPrismatic.remove(index);
                    if (selectedJointIndex >= dhModels.size()) {
                        selectedJointIndex = dhModels.size() - 1;
                    }
                    rebuildUIControls();
                    updateRobot3D();
                }
            });

            cardHeader.getChildren().addAll(title, prismaticToggle, spacer, deleteBtn);

            // Inside rebuildUIControls() method of Kinematic3DApp.java

            HBox limitsRow = new HBox(10);
            limitsRow.setAlignment(Pos.CENTER_LEFT);

            Label minLbl = new Label("Min θ:");
            minLbl.setStyle("-fx-text-fill: #abb2bf; -fx-font-size: 10px;");
            TextField minTxt = new TextField(String.format("%.0f", model.getMinTheta()));
            minTxt.setPrefWidth(50);
            minTxt.setStyle("-fx-background-color: #1e1e24; -fx-text-fill: #98c379; -fx-font-size: 10px; -fx-border-color: #4b5263; -fx-border-radius: 3;");

            minTxt.setOnAction(e -> {
                try {
                    double v = ExpressionParser.parse(minTxt.getText());
                    if (minTxt.getText().toLowerCase().contains("pi")) v = Math.toDegrees(v);
                    model.minThetaProperty().set(v);
                } catch (Exception ignored) {}
            });

            Label maxLbl = new Label("Max θ:");
            maxLbl.setStyle("-fx-text-fill: #abb2bf; -fx-font-size: 10px;");
            TextField maxTxt = new TextField(String.format("%.0f", model.getMaxTheta()));
            maxTxt.setPrefWidth(50);
            maxTxt.setStyle("-fx-background-color: #1e1e24; -fx-text-fill: #98c379; -fx-font-size: 10px; -fx-border-color: #4b5263; -fx-border-radius: 3;");

            maxTxt.setOnAction(e -> {
                try {
                    double v = ExpressionParser.parse(maxTxt.getText());
                    if (maxTxt.getText().toLowerCase().contains("pi")) v = Math.toDegrees(v);
                    model.maxThetaProperty().set(v);
                } catch (Exception ignored) {}
            });

            limitsRow.getChildren().addAll(minLbl, minTxt, maxLbl, maxTxt);

            HBox dialsRow = new HBox(30);
            dialsRow.setAlignment(Pos.CENTER);

            String thetaLabelText = isPrismatic ? "Angle (θ) [fixed]" : "▶ Angle (θ) [driven]";
            VBox thetaBox = new VBox(5, new Label(thetaLabelText), new RotaryDial(model.thetaProperty(), model.getMinTheta(), model.getMaxTheta()));
            thetaBox.setAlignment(Pos.CENTER);

            VBox alphaBox = new VBox(5, new Label("Twist (α)"), new RotaryDial(model.alphaProperty(), -180, 180));
            alphaBox.setAlignment(Pos.CENTER);

            dialsRow.getChildren().addAll(thetaBox, alphaBox);

            String dLabelText = isPrismatic ? "▶ d (drive):" : "d (Offset):";

            card.getChildren().addAll(
                    cardHeader,
                    createSliderRow("a (Length):", -20, 20, model.aProperty(), false),
                    createSliderRow(dLabelText, -20, 20, model.dProperty(), false),
                    dialsRow, // Replaced the two angle sliders with the dual rotary dials
                    limitsRow
            );

            controlsContainer.getChildren().add(card);
        }
    }

    private HBox createSliderRow(String label, double min, double max, DoubleProperty prop, boolean allowPiDegrees) {
        HBox row = new HBox(8);
        row.setAlignment(Pos.CENTER_LEFT);

        Label lbl = new Label(label);
        lbl.setPrefWidth(75);
        lbl.setStyle("-fx-text-fill: #abb2bf; -fx-font-size: 11px;");

        Slider slider = new Slider(min, max, prop.get());
        HBox.setHgrow(slider, Priority.ALWAYS);

        TextField txtInput = new TextField(String.format("%.2f", prop.get()));
        txtInput.setPrefWidth(65);
        txtInput.setStyle("-fx-background-color: #1e1e24; -fx-text-fill: #e5c07b; -fx-font-size: 11px; -fx-border-color: #4b5263; -fx-border-radius: 3;");

        slider.valueProperty().addListener((obs, oldVal, newVal) -> {
            if (!txtInput.isFocused()) {
                prop.set(newVal.doubleValue());
                txtInput.setText(String.format("%.2f", newVal.doubleValue()));
                updateRobot3D();
            }
        });

        Runnable applyTextVal = () -> {
            try {
                String rawText = txtInput.getText();
                double parsedVal = ExpressionParser.parse(rawText);

                if (allowPiDegrees && rawText.toLowerCase().contains("pi")) {
                    parsedVal = Math.toDegrees(parsedVal);
                }

                prop.set(parsedVal);
                slider.setValue(parsedVal);
                txtInput.setStyle("-fx-background-color: #1e1e24; -fx-text-fill: #e5c07b; -fx-font-size: 11px; -fx-border-color: #4b5263; -fx-border-radius: 3;");
                updateRobot3D();
            } catch (Exception ex) {
                txtInput.setStyle("-fx-background-color: #1e1e24; -fx-text-fill: #e06c75; -fx-font-size: 11px; -fx-border-color: #e06c75; -fx-border-radius: 3;");
            }
        };

        txtInput.setOnAction(e -> applyTextVal.run());
        txtInput.focusedProperty().addListener((obs, wasFocused, isFocused) -> {
            if (!isFocused) applyTextVal.run();
        });

        row.getChildren().addAll(lbl, slider, txtInput);
        return row;
    }

    /**
     * STRUCTURAL rebuild: tears down and recreates the per-joint 3D nodes (axis markers, link
     * cylinders, selection highlight). This allocates new geometry, so it's only called on
     * structural changes - adding/removing a joint, loading a preset, importing, selecting a
     * different joint, or a manual slider/dial edit. It is deliberately NOT called from the
     * 60fps animation loop; see refreshJointTransforms() for that.
     */
    private void updateRobot3D() {
        jointsGroup.getChildren().clear();
        jointAxisNodes.clear();
        linkCylinderNodes.clear();
        highlightBoxNode = null;

        List<Matrix4x4> transforms = computeCurrentTransforms();

        updateHudDisplay(transforms);

        for (int i = 0; i < transforms.size(); i++) {
            Matrix4x4 mat = transforms.get(i);
            boolean belowFloor = mat.getPosition()[2] < -FLOOR_EPSILON;

            AxisGroup axis = new AxisGroup(3.0, 0.2, i);
            axis.setUserData(i);
            applyMatrixToNode(axis, mat);
            jointsGroup.getChildren().add(axis);
            jointAxisNodes.add(axis);

            if (i == selectedJointIndex) {
                highlightBoxNode = createSelectionHighlightBox(4.5);
                applyMatrixToNode(highlightBoxNode, mat);
                jointsGroup.getChildren().add(highlightBoxNode);
            }

            if (i > 0) {
                double[] pPrev = transforms.get(i - 1).getPosition();
                double[] pCurr = mat.getPosition();

                Point3D start = new Point3D(pPrev[0], pPrev[1], pPrev[2]);
                Point3D end = new Point3D(pCurr[0], pCurr[1], pCurr[2]);

                // Flag the link red if either endpoint has dipped below the floor - a visible,
                // immediate cue on top of the floor plane itself.
                boolean prevBelow = pPrev[2] < -FLOOR_EPSILON;
                Color linkColor = (belowFloor || prevBelow) ? Color.web("#e06c75") : Color.GRAY;

                Node linkNode = createLinkCylinder(start, end, 0.4, linkColor);
                jointsGroup.getChildren().add(linkNode);
                linkCylinderNodes.add(linkNode instanceof Cylinder ? (Cylinder) linkNode : null);
            }
        }
    }

    /**
     * FAST PATH: updates only the transforms of already-existing joint/link nodes - no new
     * geometry is allocated. Safe to call every animation frame. Only theta/alpha change
     * during playback (link lengths 'a'/'d' stay fixed), so reusing each Cylinder's existing
     * mesh and just repositioning/reorienting it is valid and cheap.
     */
    private void refreshJointTransforms() {
        List<Matrix4x4> transforms = computeCurrentTransforms();

        updateHudDisplay(transforms);

        for (int i = 0; i < transforms.size() && i < jointAxisNodes.size(); i++) {
            Matrix4x4 mat = transforms.get(i);
            replaceMatrixOnNode(jointAxisNodes.get(i), mat);

            if (i == selectedJointIndex && highlightBoxNode != null) {
                replaceMatrixOnNode(highlightBoxNode, mat);
            }

            if (i > 0 && (i - 1) < linkCylinderNodes.size()) {
                double[] pPrev = transforms.get(i - 1).getPosition();
                double[] pCurr = mat.getPosition();
                Point3D start = new Point3D(pPrev[0], pPrev[1], pPrev[2]);
                Point3D end = new Point3D(pCurr[0], pCurr[1], pCurr[2]);
                updateLinkTransform(linkCylinderNodes.get(i - 1), start, end);
            }
        }
    }

    private void applyMatrixToNode(Node node, Matrix4x4 m) {
        Affine affine = new Affine(
                m.get(0, 0), m.get(0, 1), m.get(0, 2), m.get(0, 3),
                m.get(1, 0), m.get(1, 1), m.get(1, 2), m.get(1, 3),
                m.get(2, 0), m.get(2, 1), m.get(2, 2), m.get(2, 3)
        );
        node.getTransforms().add(affine);
    }

    /** Like applyMatrixToNode, but replaces the node's existing transform instead of stacking. */
    private void replaceMatrixOnNode(Node node, Matrix4x4 m) {
        Affine affine = new Affine(
                m.get(0, 0), m.get(0, 1), m.get(0, 2), m.get(0, 3),
                m.get(1, 0), m.get(1, 1), m.get(1, 2), m.get(1, 3),
                m.get(2, 0), m.get(2, 1), m.get(2, 2), m.get(2, 3)
        );
        node.getTransforms().setAll(affine);
    }

    /** Repositions/reorients an existing link Cylinder between two points without reallocating it. */
    private void updateLinkTransform(Cylinder cylinder, Point3D p1, Point3D p2) {
        if (cylinder == null) return;
        Point3D diff = p2.subtract(p1);
        if (diff.magnitude() < 1e-4) return;

        Point3D mid = p1.add(p2).multiply(0.5);
        Point3D yAxis = new Point3D(0, 1, 0);
        Point3D axisOfRot = yAxis.crossProduct(diff);
        double angle = yAxis.angle(diff);

        cylinder.getTransforms().clear();
        cylinder.getTransforms().add(new Translate(mid.getX(), mid.getY(), mid.getZ()));
        if (axisOfRot.magnitude() > 1e-4) {
            cylinder.getTransforms().add(new Rotate(angle, axisOfRot));
        }
    }

    private Node createLinkCylinder(Point3D p1, Point3D p2, double radius, Color color) {
        Point3D diff = p2.subtract(p1);
        double length = diff.magnitude();

        if (length < 1e-4) return new Group();

        Point3D mid = p1.add(p2).multiply(0.5);
        Cylinder cylinder = new Cylinder(radius, length);
        cylinder.setMaterial(new PhongMaterial(color));

        Point3D yAxis = new Point3D(0, 1, 0);
        Point3D axisOfRot = yAxis.crossProduct(diff);
        double angle = yAxis.angle(diff);

        cylinder.getTransforms().add(new Translate(mid.getX(), mid.getY(), mid.getZ()));

        if (axisOfRot.magnitude() > 1e-4) {
            cylinder.getTransforms().add(new Rotate(angle, axisOfRot));
        }

        return cylinder;
    }

    private void loadScaraPreset() {
        dhModels.clear();
        jointIsPrismatic.clear();
        // Joint 1: Base turn [-120°, 120°]
        dhModels.add(new DHParameterModel(10.0, 0.0, 0.0, 0.0, -120.0, 120.0));
        jointIsPrismatic.add(false);
        // Joint 2: Elbow [-110°, 110°] prevents link foldback collision
        dhModels.add(new DHParameterModel(8.0, 180.0, 0.0, 0.0, -110.0, 110.0));
        jointIsPrismatic.add(false);
        // Joint 3: Prismatic translation offset - actually driven by d now, not theta
        dhModels.add(new DHParameterModel(0.0, 0.0, 5.0, 0.0, -90.0, 90.0));
        jointIsPrismatic.add(true);
        selectedJointIndex = 0;
        rebuildUIControls();
        updateRobot3D();
    }

    private void loadPuma560Preset() {
        dhModels.clear();
        jointIsPrismatic.clear();
        dhModels.add(new DHParameterModel(0.0, -90.0, 0.0, 0.0, -160.0, 160.0));
        dhModels.add(new DHParameterModel(8.0, 0.0, 0.0, -30.0, -120.0, 120.0));
        dhModels.add(new DHParameterModel(2.0, 90.0, 0.0, 45.0, -135.0, 135.0));
        dhModels.add(new DHParameterModel(0.0, -90.0, 8.0, 0.0, -140.0, 140.0));
        dhModels.add(new DHParameterModel(0.0, 90.0, 0.0, 30.0, -100.0, 100.0));
        dhModels.add(new DHParameterModel(0.0, 0.0, 2.0, 0.0, -180.0, 180.0));
        jointIsPrismatic.addAll(List.of(false, false, false, false, false, false));
        selectedJointIndex = 0;
        rebuildUIControls();
        updateRobot3D();
    }

    private void exportToJson(Stage stage) {
        FileChooser fileChooser = new FileChooser();
        fileChooser.setTitle("Export DH Table to JSON");
        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("JSON Files (*.json)", "*.json"));
        fileChooser.setInitialFileName("robot_dh_config.json");

        File file = fileChooser.showSaveDialog(stage);
        if (file == null) return;

        StringBuilder json = new StringBuilder("[\n");
        for (int i = 0; i < dhModels.size(); i++) {
            DHParameterModel model = dhModels.get(i);
            String type = jointIsPrismatic.get(i) ? "prismatic" : "revolute";
            json.append(String.format("  { \"a\": %.4f, \"alpha\": %.4f, \"d\": %.4f, \"theta\": %.4f, \"type\": \"%s\" }",
                    model.getA(), model.getAlpha(), model.getD(), model.getTheta(), type));
            if (i < dhModels.size() - 1) json.append(",");
            json.append("\n");
        }
        json.append("]");

        try (PrintWriter writer = new PrintWriter(file)) {
            writer.write(json.toString());
        } catch (Exception e) {
            showErrorDialog("Export Error", "Failed to save file: " + e.getMessage());
        }
    }

    private void importFromJson(Stage stage) {
        FileChooser fileChooser = new FileChooser();
        fileChooser.setTitle("Import DH Table from JSON");
        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("JSON Files (*.json)", "*.json"));

        File file = fileChooser.showOpenDialog(stage);
        if (file == null) return;

        try {
            String content = Files.readString(file.toPath()).trim();
            if (!content.startsWith("[") || !content.endsWith("]")) {
                throw new IllegalArgumentException("Invalid JSON format.");
            }

            List<DHParameterModel> newModels = new ArrayList<>();
            List<Boolean> newTypes = new ArrayList<>();
            String inner = content.substring(1, content.length() - 1).trim();
            String[] objects = inner.split("(?<=\\}),\\s*(?=\\{)");

            for (String objStr : objects) {
                double a = extractJsonDouble(objStr, "a");
                double alpha = extractJsonDouble(objStr, "alpha");
                double d = extractJsonDouble(objStr, "d");
                double theta = extractJsonDouble(objStr, "theta");
                newModels.add(new DHParameterModel(a, alpha, d, theta));
                newTypes.add("prismatic".equals(extractJsonString(objStr, "type")));
            }

            if (!newModels.isEmpty()) {
                dhModels.clear();
                dhModels.addAll(newModels);
                jointIsPrismatic.clear();
                jointIsPrismatic.addAll(newTypes);
                selectedJointIndex = 0;
                rebuildUIControls();
                updateRobot3D();
            }
        } catch (Exception e) {
            showErrorDialog("Import Error", "Failed to load DH table: " + e.getMessage());
        }
    }

    private double extractJsonDouble(String jsonObj, String key) {
        java.util.regex.Pattern pattern = java.util.regex.Pattern.compile("\"" + key + "\"\\s*:\\s*([-+]?[0-9]*\\.?[0-9]+)");
        java.util.regex.Matcher matcher = pattern.matcher(jsonObj);
        return matcher.find() ? Double.parseDouble(matcher.group(1)) : 0.0;
    }

    /** Returns the string value for key, or null if absent - used so older exports without "type" still import fine. */
    private String extractJsonString(String jsonObj, String key) {
        java.util.regex.Pattern pattern = java.util.regex.Pattern.compile("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
        java.util.regex.Matcher matcher = pattern.matcher(jsonObj);
        return matcher.find() ? matcher.group(1) : null;
    }

    private void showErrorDialog(String title, String message) {
        Alert alert = new Alert(Alert.AlertType.ERROR);
        alert.setTitle(title);
        alert.setHeaderText(null);
        alert.setContentText(message);
        alert.showAndWait();
    }

    public static void main(String[] args) {
        launch(args);
    }
}