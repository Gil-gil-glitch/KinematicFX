package com.example.dhparameterfx;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.geometry.Insets;
import javafx.geometry.Point3D;
import javafx.geometry.Pos;
import javafx.scene.*;
import javafx.scene.control.*;
import javafx.scene.input.PickResult;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;

import java.util.ArrayList;
import java.util.List;

/**
 * Standalone visual test bench for {@link DifferentialDriveKinematics} and
 * {@link MobileRobotModel}. Deliberately separate from {@code kinematic3DApp}
 * for now (per the plan to keep the mobile base independent of the arm's
 * dhModels chain until both are proven out).
 * <p>
 * Render convention: robot's planar pose (x, y) maps to world (X, Z), with
 * Y as the vertical/up axis — the usual "ground plane" layout for a
 * top-down/orbiting 3D view. Pose theta = 0 (forward = +x in the pure-math
 * model) is rendered as the chassis's local +X axis, so the chassis group is
 * rotated about Y by -thetaDeg each frame.
 */
public class MobileRobotTestApp extends Application {

    private static final double SCALE = 20.0; // world units per pose unit

    private final MobileRobotModel robot = new MobileRobotModel(1.0, 3.0, 6.0);

    private final Group world = new Group();
    private final Group robotVisual = new Group();
    private final Group robotGeometryGroup = new Group();
    private final Group trailGroup = new Group();
    private Box floorPlane;

    private Label hudLabel;
    private Label selfTestLabel;

    private boolean driveToTargetMode = false;
    private double targetX = 0, targetY = 0;
    private final Sphere targetMarker = new Sphere(0.3 * SCALE / 10.0);

    private SubScene subScene;
    private OrbitCamera cameraRig;
    private PerspectiveCamera fpvCamera;
    private boolean fpvActive = false;

    private AnimationTimer timer;
    private long lastNanos = -1;
    private int frameCounter = 0;

    @Override
    public void start(Stage stage) {

        subScene = new SubScene(world, 800, 700, true, SceneAntialiasing.BALANCED);
        subScene.setFill(Color.web("#1e1e24"));

        cameraRig = new OrbitCamera();
        subScene.setCamera(cameraRig.getCamera());
        world.getChildren().add(cameraRig.getRootNode());

        floorPlane = buildFloorPlane(400, 20);
        world.getChildren().add(floorPlane);

        trailGroup.setMouseTransparent(true);
        world.getChildren().add(trailGroup);

        // robotVisual carries the pose transform each frame (see
        // updateRobotVisualTransform). robotGeometryGroup holds the
        // rebuildable chassis/wheel/arrow meshes; fpvCamera is a *separate*
        // permanent child so buildRobotVisual()'s clear-and-rebuild (on
        // wheel-radius/track-width changes) never removes the camera.
        buildRobotVisual();
        robotVisual.getChildren().add(robotGeometryGroup);

        fpvCamera = new PerspectiveCamera(true);
        fpvCamera.setNearClip(0.05);
        fpvCamera.setFarClip(2000);
        fpvCamera.setFieldOfView(75);
        // Camera's default forward is local +Z; rotate 90 deg about Y so it
        // looks down local +X, which is this robot's forward axis (see the
        // convention note in buildRobotVisual/updateRobotVisualTransform).
        fpvCamera.getTransforms().add(new Rotate(90, Rotate.Y_AXIS));
        fpvCamera.setTranslateX(robot.getWheelRadius() * SCALE / 10.0 * 0.6);
        fpvCamera.setTranslateY(-robot.getWheelRadius() * SCALE / 10.0 * 1.6);
        robotVisual.getChildren().add(fpvCamera);

        world.getChildren().add(robotVisual);

        targetMarker.setMaterial(new PhongMaterial(Color.web("#e5c07b")));
        targetMarker.setVisible(false);
        world.getChildren().add(targetMarker);

        AmbientLight ambient = new AmbientLight(Color.color(0.45, 0.45, 0.45));
        PointLight light = new PointLight(Color.WHITE);
        light.setTranslateX(-30);
        light.setTranslateY(-60);
        light.setTranslateZ(-30);
        world.getChildren().addAll(ambient, light);

        subScene.setOnMouseClicked(e -> {
            PickResult pr = e.getPickResult();
            if (pr.getIntersectedNode() == floorPlane) {
                Point3D p = pr.getIntersectedPoint();
                targetX = p.getX() / SCALE;
                targetY = p.getZ() / SCALE;
                driveToTargetMode = true;
                targetMarker.setTranslateX(p.getX());
                targetMarker.setTranslateY(0);
                targetMarker.setTranslateZ(p.getZ());
                targetMarker.setVisible(true);
            }
        });

        StackPane viewportPane = new StackPane(subScene);
        subScene.widthProperty().bind(viewportPane.widthProperty());
        subScene.heightProperty().bind(viewportPane.heightProperty());

        BorderPane root = new BorderPane();
        root.setCenter(viewportPane);
        root.setRight(buildControlPanel());

        Scene scene = new Scene(root, 1150, 750);
        cameraRig.registerMouseEvents(scene);

        stage.setTitle("Mobile Robot Kinematics Test Bench");
        stage.setScene(scene);
        stage.show();

        startLoop();
    }

    // ---------------------------------------------------------------
    // Scene construction
    // ---------------------------------------------------------------

    private Box buildFloorPlane(double extent, double gridSpacing) {
        Box floor = new Box(extent, 0.4, extent);
        PhongMaterial mat = new PhongMaterial(Color.web("#2b2b36"));
        floor.setMaterial(mat);
        floor.setTranslateY(0.2);
        return floor;
    }

    private void buildRobotVisual() {
        robotGeometryGroup.getChildren().clear();

        double r = robot.getWheelRadius() * SCALE / 10.0;
        double trackW = robot.getTrackWidth() * SCALE / 10.0;

        // Local-frame convention (must match updateRobotVisualTransform()):
        // forward = local +X, left/right (track width) = local +/-Z.
        // Previously this method built forward along +Z while the rotation
        // transform assumed +X, which rotated the visible heading 90 degrees
        // away from the direction the robot actually integrates toward —
        // that mismatch is what produced the sideways-sliding look.
        Box chassis = new Box(r * 2.2, r * 0.6, trackW * 0.9);
        chassis.setMaterial(new PhongMaterial(Color.web("#61afef")));
        chassis.setTranslateY(-r * 0.3);

        // Wheel axle must run along local Z (perpendicular to forward/+X),
        // so rotate the cylinder's default Y-axis onto Z via a 90 deg
        // rotation about X, not Z.
        Cylinder leftWheel = new Cylinder(r, r * 0.5);
        leftWheel.setMaterial(new PhongMaterial(Color.web("#3b3b4d")));
        leftWheel.getTransforms().add(new Rotate(90, Rotate.X_AXIS));
        leftWheel.setTranslateZ(-trackW / 2.0);

        Cylinder rightWheel = new Cylinder(r, r * 0.5);
        rightWheel.setMaterial(new PhongMaterial(Color.web("#3b3b4d")));
        rightWheel.getTransforms().add(new Rotate(90, Rotate.X_AXIS));
        rightWheel.setTranslateZ(trackW / 2.0);

        Box headingArrow = new Box(r * 1.4, r * 0.25, r * 0.25);
        headingArrow.setMaterial(new PhongMaterial(Color.web("#e06c75")));
        headingArrow.setTranslateX(r * 1.6);
        headingArrow.setTranslateY(-r * 0.3);

        robotGeometryGroup.getChildren().addAll(chassis, leftWheel, rightWheel, headingArrow);
    }

    /** Re-derives the FPV camera's mount offset from the current wheel radius. */
    private void repositionFpvCamera() {
        if (fpvCamera == null) return;
        double r = robot.getWheelRadius() * SCALE / 10.0;
        fpvCamera.setTranslateX(r * 0.6);
        fpvCamera.setTranslateY(-r * 1.6);
    }

    private void updateRobotVisualTransform() {
        Pose2D pose = robot.getPose();
        robotVisual.getTransforms().clear();
        robotVisual.getTransforms().add(new Translate(pose.x() * SCALE, 0, pose.y() * SCALE));
        robotVisual.getTransforms().add(new Rotate(-pose.thetaDeg(), Rotate.Y_AXIS));
    }

    private void addTrailMarker() {
        Pose2D pose = robot.getPose();
        Sphere dot = new Sphere(0.6);
        dot.setMaterial(new PhongMaterial(Color.web("#98c379")));
        dot.setTranslateX(pose.x() * SCALE);
        dot.setTranslateY(0);
        dot.setTranslateZ(pose.y() * SCALE);
        trailGroup.getChildren().add(dot);
        if (trailGroup.getChildren().size() > 600) {
            trailGroup.getChildren().remove(0);
        }
    }

    // ---------------------------------------------------------------
    // Animation loop
    // ---------------------------------------------------------------

    private void startLoop() {
        timer = new AnimationTimer() {
            @Override
            public void handle(long now) {
                if (lastNanos < 0) {
                    lastNanos = now;
                    return;
                }
                double dt = Math.min(0.05, (now - lastNanos) / 1e9);
                lastNanos = now;

                double v;
                double omega;

                if (driveToTargetMode) {
                    double[] cmd = goToGoalController();
                    v = cmd[0];
                    omega = cmd[1];
                    double dx = targetX - robot.getPose().x();
                    double dy = targetY - robot.getPose().y();
                    if (Math.hypot(dx, dy) < 0.3) {
                        driveToTargetMode = false;
                    }
                } else {
                    v = teleopV.getValue();
                    omega = teleopOmega.getValue();
                }

                robot.step(v, omega, dt);
                updateRobotVisualTransform();

                frameCounter++;
                if (frameCounter % 4 == 0) {
                    addTrailMarker();
                }

                updateHud();
            }
        };
        timer.start();
    }

    /** Simple proportional go-to-goal unicycle controller: turn toward the goal, then drive. */
    private double[] goToGoalController() {
        Pose2D pose = robot.getPose();
        double dx = targetX - pose.x();
        double dy = targetY - pose.y();
        double dist = Math.hypot(dx, dy);

        double desiredHeading = Math.atan2(dy, dx);
        double headingErr = normalizeAngleRad(desiredHeading - pose.thetaRad());

        double kV = 1.2, kW = 2.5;
        double v = Math.min(4.0, kV * dist);
        // Slow down the forward speed while turning to face the goal, like a real go-to-goal controller.
        v *= Math.max(0.0, 1.0 - Math.abs(headingErr) / Math.PI);
        double omega = kW * headingErr;

        return new double[]{v, omega};
    }

    private double normalizeAngleRad(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private void updateHud() {
        Pose2D pose = robot.getPose();
        hudLabel.setText(String.format(
                "x: %6.2f   y: %6.2f   theta: %6.1f deg%n" +
                        "wheel L: %5.2f rad/s   wheel R: %5.2f rad/s%n" +
                        "mode: %s",
                pose.x(), pose.y(), pose.thetaDeg(),
                robot.getLeftWheelSpeed(), robot.getRightWheelSpeed(),
                driveToTargetMode
                        ? String.format("drive-to-target (%.1f, %.1f)", targetX, targetY)
                        : "teleop"
        ));
    }

    // ---------------------------------------------------------------
    // Self-test: round-trips (v, omega) through inverse -> forward kinematics
    // ---------------------------------------------------------------

    private void runSelfTest() {
        DifferentialDriveKinematics k = new DifferentialDriveKinematics();
        DifferentialDriveKinematics.Chassis chassis = new DifferentialDriveKinematics.Chassis(
                robot.getWheelRadius(), robot.getTrackWidth(), 1000.0); // huge cap: test the math, not clamping

        double[][] samples = {{3.0, 0.0}, {0.0, 1.5}, {2.5, 0.8}, {-2.0, -0.5}, {5.0, 2.0}};
        double worst = 0.0;

        for (double[] s : samples) {
            WheelSpeeds wheels = k.computeWheelSpeeds(s[0], s[1], chassis);
            DifferentialDriveKinematics.BodyVelocity back = k.computeBodyVelocity(wheels, chassis);
            double diff = Math.max(Math.abs(back.v() - s[0]), Math.abs(back.omega() - s[1]));
            worst = Math.max(worst, diff);
        }

        // Also check that a pure-rotation command (v=0) never moves x/y over a short integration.
        Pose2D start = new Pose2D(2.0, -1.0, 30.0);
        Pose2D afterRotation = k.integrate(start, 0.0, 1.2, 0.5);
        double posDrift = Math.hypot(afterRotation.x() - start.x(), afterRotation.y() - start.y());

        if (worst < 1e-9 && posDrift < 1e-9) {
            selfTestLabel.setText(String.format(
                    "\u2713 Inverse/forward kinematics round-trip OK (max err %.2e), pure rotation has zero position drift.",
                    worst));
            selfTestLabel.setStyle("-fx-text-fill: #98c379; -fx-font-size: 11px; -fx-font-weight: bold;");
        } else {
            selfTestLabel.setText(String.format(
                    "\u26A0 Round-trip error %.2e, rotation drift %.2e — check kinematics signs/units.",
                    worst, posDrift));
            selfTestLabel.setStyle("-fx-text-fill: #e5c07b; -fx-font-size: 11px; -fx-font-weight: bold;");
        }
    }

    // ---------------------------------------------------------------
    // Control panel
    // ---------------------------------------------------------------

    private Slider teleopV;
    private Slider teleopOmega;

    private VBox buildControlPanel() {
        VBox panel = new VBox(10);
        panel.setPrefWidth(340);
        panel.setPadding(new Insets(15));
        panel.setStyle("-fx-background-color: #2b2b36;");

        Label header = new Label("Mobile Robot (Differential Drive)");
        header.setStyle("-fx-font-size: 15px; -fx-font-weight: bold; -fx-text-fill: white;");

        Label chassisLabel = new Label("Chassis geometry:");
        chassisLabel.setStyle("-fx-font-size: 11px; -fx-text-fill: #abb2bf; -fx-font-weight: bold;");

        HBox wheelRadiusRow = boundSliderRow("Wheel radius:", 0.3, 3.0, robot.wheelRadiusProperty());
        HBox trackWidthRow = boundSliderRow("Track width:", 1.0, 8.0, robot.trackWidthProperty());
        HBox maxSpeedRow = boundSliderRow("Max wheel speed:", 1.0, 15.0, robot.maxWheelSpeedRadPerSecProperty());

        wheelRadiusRow.getChildren().get(1).setOnMouseReleased(e -> buildRobotVisual());
        // Rebuild the visual whenever geometry properties change, not just on release.
        robot.wheelRadiusProperty().addListener((o, ov, nv) -> {
            buildRobotVisual();
            repositionFpvCamera();
        });
        robot.trackWidthProperty().addListener((o, ov, nv) -> buildRobotVisual());

        Label teleopLabel = new Label("Teleop (used when not driving to a target):");
        teleopLabel.setStyle("-fx-font-size: 11px; -fx-text-fill: #abb2bf; -fx-font-weight: bold;");

        teleopV = new Slider(-8, 8, 0);
        teleopOmega = new Slider(-3, 3, 0);
        HBox teleopVRow = sliderRow("v (units/s):", teleopV);
        HBox teleopOmegaRow = sliderRow("\u03C9 (rad/s):", teleopOmega);

        Button stopBtn = new Button("Stop / cancel drive-to-target");
        stopBtn.setMaxWidth(Double.MAX_VALUE);
        stopBtn.setStyle("-fx-background-color: #e06c75; -fx-text-fill: white; -fx-font-weight: bold;");
        stopBtn.setOnAction(e -> {
            driveToTargetMode = false;
            teleopV.setValue(0);
            teleopOmega.setValue(0);
            targetMarker.setVisible(false);
        });

        Button cameraToggleBtn = new Button("Switch to 1st-person camera");
        cameraToggleBtn.setMaxWidth(Double.MAX_VALUE);
        cameraToggleBtn.setStyle("-fx-background-color: #c678dd; -fx-text-fill: white; -fx-font-weight: bold;");
        cameraToggleBtn.setOnAction(e -> {
            fpvActive = !fpvActive;
            subScene.setCamera(fpvActive ? fpvCamera : cameraRig.getCamera());
            cameraToggleBtn.setText(fpvActive ? "Switch to orbit camera" : "Switch to 1st-person camera");
        });

        Label hintLabel = new Label("Click the floor to drive there (go-to-goal controller).");
        hintLabel.setWrapText(true);
        hintLabel.setStyle("-fx-font-size: 11px; -fx-text-fill: #abb2bf;");

        Button resetBtn = new Button("Reset pose to origin");
        resetBtn.setMaxWidth(Double.MAX_VALUE);
        resetBtn.setStyle("-fx-background-color: #3b3b4d; -fx-text-fill: #abb2bf; -fx-font-size: 11px;");
        resetBtn.setOnAction(e -> {
            robot.setPose(Pose2D.origin());
            trailGroup.getChildren().clear();
            driveToTargetMode = false;
            targetMarker.setVisible(false);
        });

        Button selfTestBtn = new Button("Run kinematics self-test");
        selfTestBtn.setMaxWidth(Double.MAX_VALUE);
        selfTestBtn.setStyle("-fx-background-color: #3b3b4d; -fx-text-fill: #abb2bf; -fx-font-size: 11px;");
        selfTestBtn.setOnAction(e -> runSelfTest());

        selfTestLabel = new Label();
        selfTestLabel.setWrapText(true);
        selfTestLabel.setMaxWidth(300);

        hudLabel = new Label();
        hudLabel.setStyle("-fx-text-fill: #d0d0d0; -fx-font-family: monospace; -fx-font-size: 11px;");
        VBox hudBox = new VBox(hudLabel);
        hudBox.setStyle("-fx-background-color: #21252b; -fx-padding: 8; -fx-background-radius: 5;");

        panel.getChildren().addAll(
                header,
                chassisLabel, wheelRadiusRow, trackWidthRow, maxSpeedRow,
                new Separator(),
                teleopLabel, teleopVRow, teleopOmegaRow, stopBtn, cameraToggleBtn, hintLabel,
                new Separator(),
                resetBtn, selfTestBtn, selfTestLabel,
                new Separator(),
                hudBox
        );

        return panel;
    }

    private HBox sliderRow(String label, Slider slider) {
        HBox row = new HBox(8);
        row.setAlignment(Pos.CENTER_LEFT);
        Label lbl = new Label(label);
        lbl.setPrefWidth(90);
        lbl.setStyle("-fx-text-fill: #abb2bf; -fx-font-size: 11px;");
        HBox.setHgrow(slider, Priority.ALWAYS);
        Label valueLbl = new Label(String.format("%.2f", slider.getValue()));
        valueLbl.setPrefWidth(45);
        valueLbl.setStyle("-fx-text-fill: #e5c07b; -fx-font-size: 11px;");
        slider.valueProperty().addListener((o, ov, nv) -> valueLbl.setText(String.format("%.2f", nv.doubleValue())));
        row.getChildren().addAll(lbl, slider, valueLbl);
        return row;
    }

    private HBox boundSliderRow(String label, double min, double max, javafx.beans.property.DoubleProperty prop) {
        Slider slider = new Slider(min, max, prop.get());
        HBox row = sliderRow(label, slider);
        slider.valueProperty().addListener((o, ov, nv) -> prop.set(nv.doubleValue()));
        return row;
    }

    public static void main(String[] args) {
        launch(args);
    }
}