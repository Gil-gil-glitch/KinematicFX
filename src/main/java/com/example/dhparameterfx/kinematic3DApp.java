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
    private final Group jointsGroup = new Group();
    private final Group floorPlaneGroup = new Group();
    private final Group trailGroup = new Group();

    private final List<AxisGroup> jointAxisNodes = new ArrayList<>();
    private final List<Cylinder> linkCylinderNodes = new ArrayList<>();
    private Node highlightBoxNode;

    private final List<Boolean> jointIsPrismatic = new ArrayList<>();

    private double getJointVar(int i) {
        return jointIsPrismatic.get(i) ? dhModels.get(i).getD() : dhModels.get(i).getTheta();
    }

    private void setJointVar(int i, double value) {
        if (jointIsPrismatic.get(i)) {
            dhModels.get(i).dProperty().set(value);
        } else {
            dhModels.get(i).setTheta(value);
        }
    }

    private boolean showFloorPlane = true;
    private static final double FLOOR_EPSILON = 0.05;

    private boolean showTransformBreakdown = false;
    private VBox transformBreakdownPanel;
    private Label breakdownStatusLabel;
    private Label selfTestResultLabel;
    private final Label[][] rzLabels = new Label[4][4];
    private final Label[][] tzLabels = new Label[4][4];
    private final Label[][] txLabels = new Label[4][4];
    private final Label[][] rxLabels = new Label[4][4];

    private boolean showPlannedTrail = true;

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

        targetSphere.setMaterial(new PhongMaterial(Color.web("#e06c75")));
        robotGroup.getChildren().add(targetSphere);
        updateTargetSpherePosition();

        floorPlaneGroup.getChildren().add(buildFloorPlane(30.0, 5.0));
        floorPlaneGroup.setVisible(showFloorPlane);
        floorPlaneGroup.setMouseTransparent(true);

        trailGroup.setVisible(showPlannedTrail);
        trailGroup.setMouseTransparent(true);

        robotGroup.getChildren().addAll(floorPlaneGroup, trailGroup, jointsGroup);

        AmbientLight ambient = new AmbientLight(Color.color(0.4, 0.4, 0.4));
        PointLight pointLight = new PointLight(Color.WHITE);
        pointLight.setTranslateX(-20);
        pointLight.setTranslateY(-40);
        pointLight.setTranslateZ(-50);
        world.getChildren().addAll(ambient, pointLight);

        dhModels.add(new DHParameterModel(0.0, 90.0, 5.0, 30.0));
        dhModels.add(new DHParameterModel(10.0, 0.0, 0.0, 40.0));
        dhModels.add(new DHParameterModel(8.0, 0.0, 0.0, -25.0));
        jointIsPrismatic.addAll(List.of(false, false, false));

        hud = new MatrixDisplayHUD();
        transformBreakdownPanel = buildTransformBreakdownPanel();
        transformBreakdownPanel.setVisible(showTransformBreakdown);
        transformBreakdownPanel.setMouseTransparent(true);

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
                if (tag instanceof Integer axisIdx) {
                    // FIX: Map axisIdx (0..N) to joint index (0..N-1).
                    // Axis 0 is the base, axis 1 is Joint 1 (index 0)
                    selectedJointIndex = Math.max(0, axisIdx - 1);
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

    private void runFullChainSelfTest() {
        List<Matrix4x4> transforms = computeCurrentTransforms();
        if (transforms.isEmpty() || dhModels.isEmpty()) {
            selfTestResultLabel.setText("No joints to test.");
            selfTestResultLabel.setStyle("-fx-text-fill: #abb2bf; -fx-font-size: 11px;");
            return;
        }

        int numCandidates = buildCandidateSteps(0, 0, 0, 0).size();
        double[] worstPerCandidate = new double[numCandidates];
        int[] worstJointPerCandidate = new int[numCandidates];
        String[] candidateNames = new String[numCandidates];

        for (int i = 0; i < dhModels.size(); i++) {
            DHParameterModel model = dhModels.get(i);
            List<ConventionCandidate> candidates = buildCandidateSteps(model.getTheta(), model.getD(), model.getA(), model.getAlpha());

            // FIX: Transforms size is N+1. Joint `i` defines the step between frame `i` and `i+1`.
            double[][] prevCumulative = toArray(transforms.get(i));
            double[][] currCumulative = toArray(transforms.get(i + 1));
            double[][] actualStep = matMul4(invertRigid(prevCumulative), currCumulative);

            for (int k = 0; k < candidates.size(); k++) {
                candidateNames[k] = candidates.get(k).name;
                double diff = maxDiff4(actualStep, candidates.get(k).matrix);
                if (diff > worstPerCandidate[k]) {
                    worstPerCandidate[k] = diff;
                    worstJointPerCandidate[k] = i;
                }
            }
        }

        int bestCandidate = 0;
        for (int k = 1; k < numCandidates; k++) {
            if (worstPerCandidate[k] < worstPerCandidate[bestCandidate]) bestCandidate = k;
        }

        double bestDiff = worstPerCandidate[bestCandidate];

        if (bestDiff < 1e-3) {
            selfTestResultLabel.setText(String.format(
                    "\u2713 All %d joint(s) consistently match: %s (max discrepancy %.5f)",
                    dhModels.size(), candidateNames[bestCandidate], bestDiff));
            selfTestResultLabel.setStyle("-fx-text-fill: #98c379; -fx-font-size: 11px; -fx-font-weight: bold;");
        } else {
            selfTestResultLabel.setText(String.format(
                    "\u26A0 No tested convention fits the whole chain (closest: %s, worst joint %d off by %.3f)",
                    candidateNames[bestCandidate], worstJointPerCandidate[bestCandidate] + 1, bestDiff));
            selfTestResultLabel.setStyle("-fx-text-fill: #e5c07b; -fx-font-size: 11px; -fx-font-weight: bold;");
        }
    }

    private void updateTargetSpherePosition() {
        targetSphere.setTranslateX(targetPos[0]);
        targetSphere.setTranslateY(targetPos[1]);
        targetSphere.setTranslateZ(targetPos[2]);
    }

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

    private double[][] rotYMatrix(double alphaDeg) {
        double a = Math.toRadians(alphaDeg);
        double c = Math.cos(a), s = Math.sin(a);
        return new double[][]{{c, 0, s, 0}, {0, 1, 0, 0}, {-s, 0, c, 0}, {0, 0, 0, 1}};
    }

    private double[][] transYMatrix(double a) {
        return new double[][]{{1, 0, 0, 0}, {0, 1, 0, a}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    }

    private static class ConventionCandidate {
        final String name;
        final double[][] matrix;
        ConventionCandidate(String name, double[][] matrix) {
            this.name = name;
            this.matrix = matrix;
        }
    }

    private List<ConventionCandidate> buildCandidateSteps(double theta, double d, double a, double alpha) {
        double[][] rz = rotZMatrix(theta);
        double[][] rzNeg = rotZMatrix(-theta);
        double[][] tz = transZMatrix(d);
        double[][] tx = transXMatrix(a);
        double[][] rx = rotXMatrix(alpha);
        double[][] rxNeg = rotXMatrix(-alpha);
        double[][] ty = transYMatrix(a);
        double[][] ry = rotYMatrix(alpha);

        List<ConventionCandidate> list = new ArrayList<>();
        list.add(new ConventionCandidate("Standard DH: Rz(\u03B8)\u00B7Tz(d)\u00B7Tx(a)\u00B7Rx(\u03B1)",
                matMul4(matMul4(rz, tz), matMul4(tx, rx))));
        list.add(new ConventionCandidate("Modified DH: Rx(\u03B1)\u00B7Tx(a)\u00B7Rz(\u03B8)\u00B7Tz(d)",
                matMul4(matMul4(rx, tx), matMul4(rz, tz))));
        list.add(new ConventionCandidate("Standard DH, \u03B8 sign flipped: Rz(-\u03B8)\u00B7Tz(d)\u00B7Tx(a)\u00B7Rx(\u03B1)",
                matMul4(matMul4(rzNeg, tz), matMul4(tx, rx))));
        list.add(new ConventionCandidate("Standard DH, \u03B1 sign flipped: Rz(\u03B8)\u00B7Tz(d)\u00B7Tx(a)\u00B7Rx(-\u03B1)",
                matMul4(matMul4(rz, tz), matMul4(tx, rxNeg))));
        list.add(new ConventionCandidate("Standard DH, twist on Y instead of X: Rz(\u03B8)\u00B7Tz(d)\u00B7Ty(a)\u00B7Ry(\u03B1)",
                matMul4(matMul4(rz, tz), matMul4(ty, ry))));
        return list;
    }

    private double maxDiff4(double[][] a, double[][] b) {
        double worst = 0;
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 4; c++) {
                worst = Math.max(worst, Math.abs(a[r][c] - b[r][c]));
            }
        }
        return worst;
    }

    private double[][] identity4() {
        return new double[][]{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    }

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

    private void updateTransformBreakdown(List<Matrix4x4> transforms) {
        if (!showTransformBreakdown || transforms.isEmpty()) return;

        int idx = Math.max(0, Math.min(selectedJointIndex, dhModels.size() - 1));
        DHParameterModel model = dhModels.get(idx);

        double theta = model.getTheta(), d = model.getD(), a = model.getA(), alpha = model.getAlpha();

        double[][] rz = rotZMatrix(theta);
        double[][] tz = transZMatrix(d);
        double[][] tx = transXMatrix(a);
        double[][] rx = rotXMatrix(alpha);

        setGridValues(rzLabels, rz);
        setGridValues(tzLabels, tz);
        setGridValues(txLabels, tx);
        setGridValues(rxLabels, rx);

        // FIX: Using the correct index for transforms frame steps.
        double[][] prevCumulative = toArray(transforms.get(idx));
        double[][] currCumulative = toArray(transforms.get(idx + 1));
        double[][] actualStep = matMul4(invertRigid(prevCumulative), currCumulative);

        List<ConventionCandidate> candidates = buildCandidateSteps(theta, d, a, alpha);
        ConventionCandidate best = null;
        double bestDiff = Double.MAX_VALUE;
        for (ConventionCandidate c : candidates) {
            double diff = maxDiff4(actualStep, c.matrix);
            if (diff < bestDiff) {
                bestDiff = diff;
                best = c;
            }
        }

        if (bestDiff < 1e-3) {
            breakdownStatusLabel.setText("\u2713 Matches: " + best.name + String.format(" (max discrepancy %.5f)", bestDiff));
            breakdownStatusLabel.setStyle("-fx-text-fill: #98c379; -fx-font-size: 10px; -fx-font-weight: bold;");
        } else {
            breakdownStatusLabel.setText(String.format(
                    "\u26A0 No tested convention matches this joint (closest: %s, off by %.3f)",
                    best.name, bestDiff));
            breakdownStatusLabel.setStyle("-fx-text-fill: #e5c07b; -fx-font-size: 10px; -fx-font-weight: bold;");
        }
    }

    private List<Matrix4x4> computeCurrentTransforms() {
        List<DHParameter> dhParams = new ArrayList<>();
        for (DHParameterModel model : dhModels) {
            dhParams.add(model.toDHParameter());
        }
        return fkEngine.computeCumulativeTransforms(dhParams);
    }

    private boolean violatesFloor(List<Matrix4x4> transforms) {
        for (Matrix4x4 m : transforms) {
            if (m.getPosition()[2] < -FLOOR_EPSILON) return true;
        }
        return false;
    }

    private void updateHudDisplay(List<Matrix4x4> transforms) {
        if (transforms.isEmpty() || hud == null) return;
        int idx = Math.max(0, Math.min(selectedJointIndex, dhModels.size() - 1));
        // FIX: The frame belonging to this joint index is shifted by +1
        hud.update(transforms.get(idx + 1));
        updateTransformBreakdown(transforms);
    }

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
            for (int i = 0; i < dhModels.size(); i++) {
                setJointVar(i, qStart[i]);
            }
        }

        for (int i = 1; i < samples.size(); i++) {
            Node segment = createLinkCylinder(samples.get(i - 1), samples.get(i), 0.15, trailColor);
            trailGroup.getChildren().add(segment);
        }

        Sphere endMarker = new Sphere(0.4);
        endMarker.setMaterial(new PhongMaterial(trailColor));
        Point3D lastPoint = samples.get(samples.size() - 1);
        endMarker.setTranslateX(lastPoint.getX());
        endMarker.setTranslateY(lastPoint.getY());
        endMarker.setTranslateZ(lastPoint.getZ());
        trailGroup.getChildren().add(endMarker);
    }

    private void runIKAndAnimate() {
        if (!ikSolver.isTargetReachable(dhModels, targetPos)) {
            showWarningDialog("Unreachable Target",
                    String.format("The target position (%.1f, %.1f, %.1f) is beyond the robot's kinematic reach.",
                            targetPos[0], targetPos[1], targetPos[2]));
            return;
        }

        double[] qStart = new double[dhModels.size()];
        for (int i = 0; i < dhModels.size(); i++) {
            qStart[i] = getJointVar(i);
        }

        boolean solved = ikSolver.solve(dhModels, targetPos, 350, 0.1, jointIsPrismatic);

        if (!solved) {
            showWarningDialog("Target Unreachable",
                    "The manipulator reached its limit towards the target, but could not match the exact position due to joint constraints.");
            for (int i = 0; i < dhModels.size(); i++) {
                setJointVar(i, qStart[i]);
            }
            return;
        }

        double[] qEnd = new double[dhModels.size()];
        for (int i = 0; i < dhModels.size(); i++) {
            qEnd[i] = getJointVar(i);
        }

        for (int i = 0; i < dhModels.size(); i++) {
            setJointVar(i, qStart[i]);
        }

        List<Matrix4x4> startTransforms = computeCurrentTransforms();
        double[] startPos = startTransforms.get(startTransforms.size() - 1).getPosition();

        buildPlannedTrail(qStart, qEnd, startPos);

        for (int i = 0; i < dhModels.size(); i++) {
            setJointVar(i, qStart[i]);
        }

        if (playbackTimer != null) playbackTimer.stop();

        final long startTime = System.nanoTime();
        final double durationNs = 1_500_000_000.0;

        final double[] lastGoodQ = qStart.clone();

        playbackTimer = new AnimationTimer() {
            @Override
            public void handle(long now) {
                double elapsed = now - startTime;
                double tNorm = Math.max(0.0, Math.min(1.0, elapsed / durationNs));
                double s = 3 * tNorm * tNorm - 2 * tNorm * tNorm * tNorm;

                if (useCartesianTrajectory) {
                    double[] currentTarget = new double[]{
                            startPos[0] + s * (targetPos[0] - startPos[0]),
                            startPos[1] + s * (targetPos[1] - startPos[1]),
                            Math.max(0.0, startPos[2] + s * (targetPos[2] - startPos[2]))
                    };
                    ikSolver.solve(dhModels, currentTarget, 5, 0.1, jointIsPrismatic);
                } else {
                    double[] currentQ = TrajectoryPlanner.interpolateCubic(qStart, qEnd, tNorm);
                    for (int i = 0; i < dhModels.size(); i++) {
                        setJointVar(i, currentQ[i]);
                    }
                }

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
                    dialsRow,
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

            // FIX: Draw the highlight box on the actual coordinate frame belonging to the selected index
            if (i == selectedJointIndex + 1) {
                highlightBoxNode = createSelectionHighlightBox(4.5);
                applyMatrixToNode(highlightBoxNode, mat);
                jointsGroup.getChildren().add(highlightBoxNode);
            }

            if (i > 0) {
                double[] pPrev = transforms.get(i - 1).getPosition();
                double[] pCurr = mat.getPosition();

                Point3D start = new Point3D(pPrev[0], pPrev[1], pPrev[2]);
                Point3D end = new Point3D(pCurr[0], pCurr[1], pCurr[2]);

                boolean prevBelow = pPrev[2] < -FLOOR_EPSILON;
                Color linkColor = (belowFloor || prevBelow) ? Color.web("#e06c75") : Color.GRAY;

                Node linkNode = createLinkCylinder(start, end, 0.4, linkColor);
                jointsGroup.getChildren().add(linkNode);
                linkCylinderNodes.add(linkNode instanceof Cylinder ? (Cylinder) linkNode : null);
            }
        }
    }

    private void refreshJointTransforms() {
        List<Matrix4x4> transforms = computeCurrentTransforms();

        updateHudDisplay(transforms);

        for (int i = 0; i < transforms.size() && i < jointAxisNodes.size(); i++) {
            Matrix4x4 mat = transforms.get(i);
            replaceMatrixOnNode(jointAxisNodes.get(i), mat);

            // FIX: Refresh the highlight frame utilizing the correct +1 shift
            if (i == selectedJointIndex + 1 && highlightBoxNode != null) {
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

    private void replaceMatrixOnNode(Node node, Matrix4x4 m) {
        Affine affine = new Affine(
                m.get(0, 0), m.get(0, 1), m.get(0, 2), m.get(0, 3),
                m.get(1, 0), m.get(1, 1), m.get(1, 2), m.get(1, 3),
                m.get(2, 0), m.get(2, 1), m.get(2, 2), m.get(2, 3)
        );
        node.getTransforms().setAll(affine);
    }

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
        dhModels.add(new DHParameterModel(10.0, 0.0, 0.0, 0.0, -120.0, 120.0));
        jointIsPrismatic.add(false);
        dhModels.add(new DHParameterModel(8.0, 180.0, 0.0, 0.0, -110.0, 110.0));
        jointIsPrismatic.add(false);
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