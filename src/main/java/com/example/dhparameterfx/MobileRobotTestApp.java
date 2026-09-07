package com.example.dhparameterfx;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.geometry.Insets;
import javafx.geometry.Point3D;
import javafx.geometry.Pos;
import javafx.scene.*;
import javafx.scene.control.*;
import javafx.scene.image.PixelWriter;
import javafx.scene.image.WritableImage;
import javafx.scene.input.PickResult;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import javafx.stage.FileChooser;
import javafx.stage.Stage;

import java.io.File;
import java.io.PrintWriter;
import java.nio.file.Files;
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

    private enum DriveType { DIFFERENTIAL, ACKERMANN }
    private DriveType driveType = DriveType.DIFFERENTIAL;

    private final MobileRobotModel robot = new MobileRobotModel(1.0, 3.0, 6.0);
    private final AckermannRobotModel ackermann = new AckermannRobotModel(6.0, 3.5, 35.0, 10.0);

    /** Returns whichever model is currently driving the sim, so the animation loop, HUD, camera, etc. don't branch on drive type themselves. */
    private PlanarRobotModel activeModel() {
        return driveType == DriveType.DIFFERENTIAL ? robot : ackermann;
    }

    private final Group world = new Group();
    private final Group robotVisual = new Group();
    private final Group robotGeometryGroup = new Group();
    private final Group ackermannVisual = new Group();
    private final Group ackermannGeometryGroup = new Group();
    private Group frontLeftPivot;
    private Group frontRightPivot;
    private final Group trailGroup = new Group();
    private Box floorPlane;

    private Label hudLabel;
    private Label selfTestLabel;
    private Label ioStatusLabel;

    // Multi-waypoint path following (pure pursuit), replacing the old
    // single-goal drive-to-target as the actual driver of motion.
    private final PurePursuitController pathController = new PurePursuitController();
    private final List<PurePursuitController.Waypoint> path = new ArrayList<>();
    private int pathTargetIndex = 0;
    private boolean followingPath = false;
    private final Group pathVisualGroup = new Group();
    private static final double LOOKAHEAD_DISTANCE = 2.5; // pose units
    private static final double PATH_DESIRED_SPEED = 4.0; // units/s
    private static final double PATH_GOAL_TOLERANCE = 0.3; // pose units

    private SubScene subScene;
    private OrbitCamera cameraRig;
    private PerspectiveCamera fpvCamera;
    private PerspectiveCamera topDownCamera;
    private enum CameraMode { ORBIT, FPV, TOP_DOWN }
    private CameraMode cameraMode = CameraMode.ORBIT;
    private static final double TOP_DOWN_HEIGHT = 220.0; // world units above ground (negative Y = up in this scene)
    private Stage primaryStage;

    private AnimationTimer timer;
    private long lastNanos = -1;
    private int frameCounter = 0;

    @Override
    public void start(Stage stage) {

        this.primaryStage = stage;
        subScene = new SubScene(world, 800, 700, true, SceneAntialiasing.BALANCED);
        subScene.setFill(Color.web("#1e1e24"));

        cameraRig = new OrbitCamera();
        subScene.setCamera(cameraRig.getCamera());
        world.getChildren().add(cameraRig.getRootNode());

        floorPlane = buildFloorPlane(400, 20);
        world.getChildren().add(floorPlane);

        trailGroup.setMouseTransparent(true);
        world.getChildren().add(trailGroup);

        pathVisualGroup.setMouseTransparent(true);
        world.getChildren().add(pathVisualGroup);

        // robotVisual carries the pose transform each frame (see
        // updateRobotVisualTransform). robotGeometryGroup holds the
        // rebuildable chassis/wheel/arrow meshes; fpvCamera is a *separate*
        // permanent child so buildRobotVisual()'s clear-and-rebuild (on
        // wheel-radius/track-width changes) never removes the camera.
        buildRobotVisual();
        robotVisual.getChildren().add(robotGeometryGroup);

        buildAckermannVisual();
        ackermannVisual.getChildren().add(ackermannGeometryGroup);
        ackermannVisual.setVisible(false); // driveType starts as DIFFERENTIAL

        fpvCamera = new PerspectiveCamera(true);
        fpvCamera.setNearClip(0.05);
        fpvCamera.setFarClip(2000);
        fpvCamera.setFieldOfView(75);
        // Camera's default forward is local +Z; rotate 90 deg about Y so it
        // looks down local +X, which is this robot's forward axis (see the
        // convention note in buildRobotVisual/updateRobotVisualTransform).
        fpvCamera.getTransforms().add(new Rotate(90, Rotate.Y_AXIS));
        robotVisual.getChildren().add(fpvCamera); // starts under the active (differential) visual
        repositionFpvCamera();

        // Top-down preset: a bird's-eye view for judging the driven trail
        // and planned path against each other, and for keeping the whole
        // arena in frame while path-following. Deliberately a child of
        // `world`, not `robotVisual` — it should NOT inherit the chassis's
        // heading rotation (that would spin the whole view every time the
        // robot turns); only its X/Z position is updated each frame in
        // startLoop() to follow the robot while staying north-up.
        topDownCamera = new PerspectiveCamera(true);
        topDownCamera.setNearClip(1.0);
        topDownCamera.setFarClip(2000);
        topDownCamera.setFieldOfView(50);
        // Default forward is +Z; rotate -90 about X so it looks straight
        // down (+Y in this scene's "negative-Y-is-up" convention — see the
        // TOP_DOWN_HEIGHT field comment).
        topDownCamera.getTransforms().add(new Rotate(-90, Rotate.X_AXIS));
        topDownCamera.setTranslateY(-TOP_DOWN_HEIGHT);
        world.getChildren().add(topDownCamera);

        world.getChildren().add(robotVisual);
        world.getChildren().add(ackermannVisual);

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
                double px = p.getX() / SCALE;
                double py = p.getZ() / SCALE;

                path.add(new PurePursuitController.Waypoint(px, py));
                rebuildPathVisual();

                // Clicking always extends the queue; if nothing is currently
                // being followed, start immediately from wherever the robot
                // is now. This keeps a single click behaving like the old
                // one-shot "drive to target" while extra clicks naturally
                // become a multi-waypoint path instead of overwriting it.
                if (!followingPath) {
                    followingPath = true;
                    pathTargetIndex = 0;
                }
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
        PhongMaterial mat = new PhongMaterial(Color.WHITE); // white so the texture's own colors show unmodified
        mat.setDiffuseMap(buildFloorTexture(extent, gridSpacing));
        floor.setMaterial(mat);
        floor.setTranslateY(0.2);
        return floor;
    }

    /**
     * Procedurally generates a checkerboard-plus-coarse-grid texture for the
     * floor. A flat solid color gives an FPV/driver-seat camera almost no
     * motion cues — no texture flow, no parallax, no way to judge speed or
     * heading drift from the ground plane alone. A fine checker pattern
     * restores that (each square passing through frame gives a concrete
     * speed/direction cue), and a coarser bright grid on top of it gives a
     * fixed distance reference (like lane markings) so scale is legible
     * from both the orbit and top-down views, not just up close.
     * <p>
     * Baked as one image sized so the checker squares line up with
     * {@code gridSpacing} world units, rather than relying on JavaFX's
     * texture-repeat support (Box UV mapping stretches one image per face
     * with no built-in tiling), so the pattern reads correctly at the
     * floor's actual scale without any extra scene-graph nodes.
     */
    private WritableImage buildFloorTexture(double extent, double gridSpacing) {
        int pixelsPerSquare = 16;
        int squaresPerSide = (int) Math.round(extent / gridSpacing);
        int imageSize = pixelsPerSquare * squaresPerSide;

        WritableImage image = new WritableImage(imageSize, imageSize);
        PixelWriter writer = image.getPixelWriter();

        Color squareA = Color.web("#2b2b36");
        Color squareB = Color.web("#33333f");
        Color coarseLine = Color.web("#61afef", 0.55);
        int coarseLineEveryNSquares = 5;
        int coarseLineThicknessPx = 2;

        for (int py = 0; py < imageSize; py++) {
            int squareRow = py / pixelsPerSquare;
            for (int px = 0; px < imageSize; px++) {
                int squareCol = px / pixelsPerSquare;

                boolean onCoarseLine =
                        (squareCol % coarseLineEveryNSquares == 0 && px % pixelsPerSquare < coarseLineThicknessPx) ||
                                (squareRow % coarseLineEveryNSquares == 0 && py % pixelsPerSquare < coarseLineThicknessPx);

                Color color;
                if (onCoarseLine) {
                    color = coarseLine;
                } else {
                    color = ((squareRow + squareCol) % 2 == 0) ? squareA : squareB;
                }
                writer.setColor(px, py, color);
            }
        }

        return image;
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

    /**
     * Builds the Ackermann chassis: a longer body (scaled from wheelbase
     * rather than wheel radius, since wheelbase is what actually varies for
     * this drive type), two fixed rear wheels, and two front wheels each
     * mounted on their own pivot {@link Group} so {@link #updateAckermannSteering}
     * can rotate them independently every frame without touching the rest
     * of the geometry. Same local-frame convention as {@link #buildRobotVisual}:
     * forward = local +X, left/right = local +/-Z.
     */
    private void buildAckermannVisual() {
        ackermannGeometryGroup.getChildren().clear();

        double r = ackermann.getWheelRadius() * SCALE / 10.0;
        double wb = ackermann.getWheelbase() * SCALE / 10.0;
        double trackW = ackermann.getTrackWidth() * SCALE / 10.0;

        // Local +Z is verified to be this app's true "left" direction under
        // the Rotate(-thetaDeg, Y_AXIS) chassis-heading convention (left
        // wheels placed at +trackW/2, right wheels at -trackW/2). This
        // matters here — unlike buildRobotVisual's cosmetic left/right
        // wheel naming, which is harmless since diff-drive wheels aren't
        // independently animated — because getLeftFrontWheelAngleDeg() vs
        // getRightFrontWheelAngleDeg() must land on the geometrically
        // correct side or a left turn would visibly toe in the wrong wheel.

        Box chassis = new Box(wb * 1.15, r * 0.6, trackW * 0.85);
        chassis.setMaterial(new PhongMaterial(Color.web("#e5c07b")));
        chassis.setTranslateY(-r * 0.3);

        Cylinder rearLeft = new Cylinder(r, r * 0.5);
        rearLeft.setMaterial(new PhongMaterial(Color.web("#3b3b4d")));
        rearLeft.getTransforms().add(new Rotate(90, Rotate.X_AXIS));
        rearLeft.setTranslateX(-wb / 2.0);
        rearLeft.setTranslateZ(trackW / 2.0);

        Cylinder rearRight = new Cylinder(r, r * 0.5);
        rearRight.setMaterial(new PhongMaterial(Color.web("#3b3b4d")));
        rearRight.getTransforms().add(new Rotate(90, Rotate.X_AXIS));
        rearRight.setTranslateX(-wb / 2.0);
        rearRight.setTranslateZ(-trackW / 2.0);

        Cylinder frontLeftWheel = new Cylinder(r, r * 0.5);
        frontLeftWheel.setMaterial(new PhongMaterial(Color.web("#3b3b4d")));
        frontLeftWheel.getTransforms().add(new Rotate(90, Rotate.X_AXIS));
        frontLeftPivot = new Group(frontLeftWheel);
        frontLeftPivot.setTranslateX(wb / 2.0);
        frontLeftPivot.setTranslateZ(trackW / 2.0);

        Cylinder frontRightWheel = new Cylinder(r, r * 0.5);
        frontRightWheel.setMaterial(new PhongMaterial(Color.web("#3b3b4d")));
        frontRightWheel.getTransforms().add(new Rotate(90, Rotate.X_AXIS));
        frontRightPivot = new Group(frontRightWheel);
        frontRightPivot.setTranslateX(wb / 2.0);
        frontRightPivot.setTranslateZ(-trackW / 2.0);

        Box headingArrow = new Box(wb * 0.3, r * 0.25, r * 0.25);
        headingArrow.setMaterial(new PhongMaterial(Color.web("#e06c75")));
        headingArrow.setTranslateX(wb / 2.0 + r * 1.2);
        headingArrow.setTranslateY(-r * 0.3);

        ackermannGeometryGroup.getChildren().addAll(
                chassis, rearLeft, rearRight, frontLeftPivot, frontRightPivot, headingArrow);
    }

    /**
     * Applies the current per-wheel Ackermann steering angles (from
     * {@link AckermannRobotModel}) to the front-wheel pivot groups. Called
     * every frame while Ackermann is active — cheap (just two rotation
     * updates), unlike {@link #buildAckermannVisual} which rebuilds meshes
     * and should only run when chassis geometry actually changes.
     */
    private void updateAckermannSteering() {
        if (frontLeftPivot == null || frontRightPivot == null) return;
        // Steering rotates about the vertical (world-up) axis, which after
        // the ground-plane convention established elsewhere is world Y —
        // same axis used for the whole-chassis heading rotation.
        frontLeftPivot.getTransforms().setAll(new Rotate(-ackermann.getLeftFrontWheelAngleDeg(), Rotate.Y_AXIS));
        frontRightPivot.getTransforms().setAll(new Rotate(-ackermann.getRightFrontWheelAngleDeg(), Rotate.Y_AXIS));
    }

    private void updateAckermannVisualTransform() {
        Pose2D pose = ackermann.getPose();
        ackermannVisual.getTransforms().clear();
        ackermannVisual.getTransforms().add(new Translate(pose.x() * SCALE, 0, pose.y() * SCALE));
        ackermannVisual.getTransforms().add(new Rotate(-pose.thetaDeg(), Rotate.Y_AXIS));
    }

    /** Re-derives the FPV camera's mount offset for whichever drive type is currently active. */
    private void repositionFpvCamera() {
        if (fpvCamera == null) return;
        if (driveType == DriveType.DIFFERENTIAL) {
            double r = robot.getWheelRadius() * SCALE / 10.0;
            fpvCamera.setTranslateX(r * 0.6);
            fpvCamera.setTranslateY(-r * 1.6);
        } else {
            double r = ackermann.getWheelRadius() * SCALE / 10.0;
            double wb = ackermann.getWheelbase() * SCALE / 10.0;
            // Mount near the front of the (longer) Ackermann chassis rather
            // than at its center, closer to where a driver's seat would be.
            fpvCamera.setTranslateX(wb * 0.25);
            fpvCamera.setTranslateY(-r * 1.8);
        }
    }

    /**
     * Switches the active drive type: swaps which visual is shown, moves
     * the FPV camera to the newly active chassis, resets both models to the
     * origin, and clears the trail/path — pose is deliberately not carried
     * over between drive types (they have different kinematics entirely, so
     * "continuing from where the other one was" isn't a meaningful state to
     * preserve) rather than attempting a lossy pose translation.
     */
    private void switchDriveType(DriveType newType) {
        if (newType == driveType) return;

        Group oldVisual = driveType == DriveType.DIFFERENTIAL ? robotVisual : ackermannVisual;
        Group newVisual = newType == DriveType.DIFFERENTIAL ? robotVisual : ackermannVisual;

        oldVisual.getChildren().remove(fpvCamera);
        newVisual.getChildren().add(fpvCamera);

        driveType = newType;
        robotVisual.setVisible(driveType == DriveType.DIFFERENTIAL);
        ackermannVisual.setVisible(driveType == DriveType.ACKERMANN);

        activeModel().setPose(Pose2D.origin());
        trailGroup.getChildren().clear();
        clearPath();
        repositionFpvCamera();
        updateHud();
    }

    private void updateRobotVisualTransform() {
        Pose2D pose = robot.getPose();
        robotVisual.getTransforms().clear();
        robotVisual.getTransforms().add(new Translate(pose.x() * SCALE, 0, pose.y() * SCALE));
        robotVisual.getTransforms().add(new Rotate(-pose.thetaDeg(), Rotate.Y_AXIS));
    }

    private void addTrailMarker() {
        Pose2D pose = activeModel().getPose();
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

    /**
     * Redraws the planned-path markers (waypoint dots + connecting
     * segments) whenever the queue changes. This is the "planned" path,
     * distinct in color from {@link #trailGroup}'s green "actually driven"
     * dots, so you can visually compare the pure-pursuit-followed route
     * against the intended clicked path.
     */
    private void rebuildPathVisual() {
        pathVisualGroup.getChildren().clear();

        for (PurePursuitController.Waypoint w : path) {
            Sphere marker = new Sphere(0.4);
            marker.setMaterial(new PhongMaterial(Color.web("#e5c07b")));
            marker.setTranslateX(w.x() * SCALE);
            marker.setTranslateY(0);
            marker.setTranslateZ(w.y() * SCALE);
            pathVisualGroup.getChildren().add(marker);
        }

        for (int i = 1; i < path.size(); i++) {
            PurePursuitController.Waypoint a = path.get(i - 1);
            PurePursuitController.Waypoint b = path.get(i);
            pathVisualGroup.getChildren().add(buildSegment(
                    a.x() * SCALE, a.y() * SCALE, b.x() * SCALE, b.y() * SCALE));
        }
    }

    /** Thin flat box laid between two floor points, used to draw the planned-path line. */
    private Box buildSegment(double x1, double z1, double x2, double z2) {
        double dx = x2 - x1;
        double dz = z2 - z1;
        double length = Math.hypot(dx, dz);

        Box segment = new Box(Math.max(length, 0.01), 0.1, 0.3);
        segment.setMaterial(new PhongMaterial(Color.web("#e5c07b", 0.7)));
        segment.setTranslateX((x1 + x2) / 2.0);
        segment.setTranslateY(0.1);
        segment.setTranslateZ((z1 + z2) / 2.0);
        segment.getTransforms().add(new Rotate(-Math.toDegrees(Math.atan2(dz, dx)), Rotate.Y_AXIS));
        return segment;
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

                if (followingPath && !path.isEmpty()) {
                    PurePursuitController.Command cmd = pathController.computeCommand(
                            activeModel().getPose(), path, pathTargetIndex,
                            LOOKAHEAD_DISTANCE, PATH_DESIRED_SPEED, PATH_GOAL_TOLERANCE);
                    v = cmd.v();
                    omega = cmd.omega();
                    pathTargetIndex = cmd.targetIndex();
                    if (cmd.pathComplete()) {
                        followingPath = false;
                    }
                } else {
                    v = teleopV.getValue();
                    omega = teleopOmega.getValue();
                }

                activeModel().step(v, omega, dt);
                if (driveType == DriveType.DIFFERENTIAL) {
                    updateRobotVisualTransform();
                } else {
                    updateAckermannVisualTransform();
                    updateAckermannSteering();
                }

                if (cameraMode == CameraMode.TOP_DOWN) {
                    Pose2D pose = activeModel().getPose();
                    topDownCamera.setTranslateX(pose.x() * SCALE);
                    topDownCamera.setTranslateZ(pose.y() * SCALE);
                }

                frameCounter++;
                if (frameCounter % 4 == 0) {
                    addTrailMarker();
                }

                updateHud();
            }
        };
        timer.start();
    }

    private void updateHud() {
        Pose2D pose = activeModel().getPose();
        String mode = followingPath
                ? String.format("following path (waypoint %d/%d)", pathTargetIndex + 1, path.size())
                : "teleop";

        String driveSpecificLine = driveType == DriveType.DIFFERENTIAL
                ? String.format("wheel L: %5.2f rad/s   wheel R: %5.2f rad/s", robot.getLeftWheelSpeed(), robot.getRightWheelSpeed())
                : String.format("steering: %5.1f deg   (front L %5.1f / R %5.1f deg)",
                ackermann.getSteeringAngleDeg(), ackermann.getLeftFrontWheelAngleDeg(), ackermann.getRightFrontWheelAngleDeg());

        hudLabel.setText(String.format(
                "drive: %s%n" +
                        "x: %6.2f   y: %6.2f   theta: %6.1f deg%n" +
                        "%s%n" +
                        "mode: %s",
                driveType == DriveType.DIFFERENTIAL ? "differential" : "ackermann",
                pose.x(), pose.y(), pose.thetaDeg(),
                driveSpecificLine,
                mode
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

        // Constant-curvature arc closure: drive a full circle at constant
        // (v, omega) in small simulated steps (matching how the real
        // animation loop calls integrate() every frame, not one giant dt)
        // and confirm the exact-arc integration returns to the start pose
        // with no accumulated drift — the in-app counterpart to
        // DifferentialDriveKinematicsTest.constantCurvatureArc_returnsToStartAfterOneFullTurn.
        double arcV = 3.0, arcOmega = 1.0;
        double period = 2 * Math.PI / arcOmega;
        Pose2D arcPose = new Pose2D(0, 0, 0);
        Pose2D arcStart = arcPose;
        int arcSteps = 720;
        double arcDt = period / arcSteps;
        for (int i = 0; i < arcSteps; i++) {
            arcPose = k.integrate(arcPose, arcV, arcOmega, arcDt);
        }
        double arcDrift = Math.hypot(arcPose.x() - arcStart.x(), arcPose.y() - arcStart.y());

        // Ackermann checks, run alongside the differential-drive ones above
        // regardless of which drive type is currently active in the viewport
        // — this validates both engines every time, not just whichever one
        // happens to be on screen.
        AckermannKinematics ak = new AckermannKinematics();
        AckermannKinematics.Chassis akChassis = new AckermannKinematics.Chassis(
                ackermann.getWheelbase(), ackermann.getTrackWidth(),
                Math.toRadians(ackermann.getMaxSteeringAngleDeg()), 1000.0); // huge speed cap: test the math, not clamping

        double akWorst = 0.0;
        for (double omega : new double[]{0.0, 0.3, -0.3, 0.6}) {
            double delta = ak.computeSteeringAngle(6.0, omega, akChassis);
            AckermannKinematics.BodyVelocity back = ak.computeBodyVelocity(6.0, delta, akChassis);
            akWorst = Math.max(akWorst, Math.abs(back.omega() - omega));
        }

        // The defining Ackermann constraint: v=0 must yield omega=0 for any requested turn.
        boolean pivotCorrectlyBlocked = Math.abs(ak.computeBodyVelocity(0.0,
                ak.computeSteeringAngle(0.0, 2.0, akChassis), akChassis).omega()) < 1e-9;

        double akArcV = 3.0;
        double akDelta = Math.toRadians(15);
        AckermannKinematics.BodyVelocity akBv = ak.computeBodyVelocity(akArcV, akDelta, akChassis);
        double akPeriod = 2 * Math.PI / akBv.omega();
        Pose2D akPose = Pose2D.origin();
        int akSteps = 720;
        double akDt = akPeriod / akSteps;
        for (int i = 0; i < akSteps; i++) {
            akPose = ak.integrate(akPose, akBv.v(), akBv.omega(), akDt);
        }
        double akArcDrift = Math.hypot(akPose.x(), akPose.y());

        boolean diffOk = worst < 1e-9 && posDrift < 1e-9 && arcDrift < 1e-3;
        boolean ackermannOk = akWorst < 1e-6 && pivotCorrectlyBlocked && akArcDrift < 1e-3;

        if (diffOk && ackermannOk) {
            selfTestLabel.setText(String.format(
                    "\u2713 Both engines OK — diff-drive round-trip %.2e / arc closes within %.4f; " +
                            "Ackermann round-trip %.2e, v=0 pivot correctly blocked, arc closes within %.4f.",
                    worst, arcDrift, akWorst, akArcDrift));
            selfTestLabel.setStyle("-fx-text-fill: #98c379; -fx-font-size: 11px; -fx-font-weight: bold;");
        } else {
            selfTestLabel.setText(String.format(
                    "\u26A0 diff-drive: round-trip %.2e, rotation drift %.2e, arc drift %.4f (%s). " +
                            "Ackermann: round-trip %.2e, pivot blocked=%s, arc drift %.4f (%s).",
                    worst, posDrift, arcDrift, diffOk ? "OK" : "FAIL",
                    akWorst, pivotCorrectlyBlocked, akArcDrift, ackermannOk ? "OK" : "FAIL"));
            selfTestLabel.setStyle("-fx-text-fill: #e5c07b; -fx-font-size: 11px; -fx-font-weight: bold;");
        }
    }

    /** Cancels path-following, clears the queued waypoints, and removes their visuals. */
    private void clearPath() {
        followingPath = false;
        pathTargetIndex = 0;
        path.clear();
        pathVisualGroup.getChildren().clear();
    }

    // ---------------------------------------------------------------
    // Persistence (chassis config + pose), mirroring kinematic3DApp's
    // exportToJson/importFromJson pattern. The actual JSON read/write logic
    // lives in MobileRobotConfigIO so it stays unit-testable without a
    // FileChooser/Stage; this method is just the UI glue.
    // ---------------------------------------------------------------

    private void exportConfigToJson() {
        if (driveType != DriveType.DIFFERENTIAL) {
            setIoStatus("Export/import currently only supports differential-drive configs — Ackermann persistence isn't built yet.", false);
            return;
        }
        FileChooser fileChooser = new FileChooser();
        fileChooser.setTitle("Export Mobile Robot Config to JSON");
        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("JSON Files (*.json)", "*.json"));
        fileChooser.setInitialFileName("mobile_robot_config.json");

        File file = fileChooser.showSaveDialog(primaryStage);
        if (file == null) return;

        try (PrintWriter writer = new PrintWriter(file)) {
            MobileRobotConfigIO.MobileRobotConfig config = MobileRobotConfigIO.fromModel(robot);
            writer.write(MobileRobotConfigIO.toJson(config));
            setIoStatus("\u2713 Saved to " + file.getName(), true);
        } catch (Exception e) {
            setIoStatus("Export failed: " + e.getMessage(), false);
        }
    }

    private void importConfigFromJson() {
        if (driveType != DriveType.DIFFERENTIAL) {
            setIoStatus("Export/import currently only supports differential-drive configs — Ackermann persistence isn't built yet.", false);
            return;
        }
        FileChooser fileChooser = new FileChooser();
        fileChooser.setTitle("Import Mobile Robot Config from JSON");
        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("JSON Files (*.json)", "*.json"));

        File file = fileChooser.showOpenDialog(primaryStage);
        if (file == null) return;

        try {
            String content = Files.readString(file.toPath());
            MobileRobotConfigIO.MobileRobotConfig config = MobileRobotConfigIO.fromJson(content);
            MobileRobotConfigIO.applyTo(robot, config);

            buildRobotVisual();
            repositionFpvCamera();
            updateRobotVisualTransform();
            trailGroup.getChildren().clear();
            clearPath();

            setIoStatus("\u2713 Loaded " + file.getName(), true);
        } catch (Exception e) {
            setIoStatus("Import failed: " + e.getMessage(), false);
        }
    }

    private void setIoStatus(String text, boolean success) {
        ioStatusLabel.setText(text);
        ioStatusLabel.setStyle("-fx-font-size: 11px; -fx-text-fill: " + (success ? "#98c379" : "#e06c75") + ";");
    }

    // ---------------------------------------------------------------
    // Control panel
    // ---------------------------------------------------------------

    private Slider teleopV;
    private Slider teleopOmega;

    private Label headerLabel;
    private VBox diffChassisBox;
    private VBox ackermannChassisBox;

    private VBox buildControlPanel() {
        VBox panel = new VBox(10);
        panel.setPrefWidth(340);
        panel.setPadding(new Insets(15));
        panel.setStyle("-fx-background-color: #2b2b36;");

        headerLabel = new Label("Mobile Robot (Differential Drive)");
        headerLabel.setStyle("-fx-font-size: 15px; -fx-font-weight: bold; -fx-text-fill: white;");

        // --- Drive-type switcher ---
        Label driveTypeLabel = new Label("Drive type:");
        driveTypeLabel.setStyle("-fx-font-size: 11px; -fx-text-fill: #abb2bf; -fx-font-weight: bold;");

        ToggleGroup driveTypeGroup = new ToggleGroup();
        RadioButton diffRadio = new RadioButton("Differential drive");
        RadioButton ackermannRadio = new RadioButton("Ackermann steering");
        diffRadio.setToggleGroup(driveTypeGroup);
        ackermannRadio.setToggleGroup(driveTypeGroup);
        diffRadio.setSelected(true);
        diffRadio.setStyle("-fx-text-fill: #abb2bf; -fx-font-size: 12px;");
        ackermannRadio.setStyle("-fx-text-fill: #abb2bf; -fx-font-size: 12px;");
        HBox driveTypeRow = new HBox(15, diffRadio, ackermannRadio);

        // --- Differential-drive chassis section ---
        Label diffChassisLabel = new Label("Chassis geometry:");
        diffChassisLabel.setStyle("-fx-font-size: 11px; -fx-text-fill: #abb2bf; -fx-font-weight: bold;");

        HBox wheelRadiusRow = boundSliderRow("Wheel radius:", 0.3, 3.0, robot.wheelRadiusProperty());
        HBox trackWidthRow = boundSliderRow("Track width:", 1.0, 8.0, robot.trackWidthProperty());
        HBox maxSpeedRow = boundSliderRow("Max wheel speed:", 1.0, 15.0, robot.maxWheelSpeedRadPerSecProperty());

        robot.wheelRadiusProperty().addListener((o, ov, nv) -> {
            buildRobotVisual();
            if (driveType == DriveType.DIFFERENTIAL) repositionFpvCamera();
        });
        robot.trackWidthProperty().addListener((o, ov, nv) -> buildRobotVisual());

        diffChassisBox = new VBox(6, diffChassisLabel, wheelRadiusRow, trackWidthRow, maxSpeedRow);

        // --- Ackermann chassis section ---
        Label akChassisLabel = new Label("Chassis geometry:");
        akChassisLabel.setStyle("-fx-font-size: 11px; -fx-text-fill: #abb2bf; -fx-font-weight: bold;");

        HBox wheelbaseRow = boundSliderRow("Wheelbase:", 2.0, 12.0, ackermann.wheelbaseProperty());
        HBox akTrackWidthRow = boundSliderRow("Track width:", 1.0, 6.0, ackermann.trackWidthProperty());
        HBox maxSteerRow = boundSliderRow("Max steering angle (deg):", 10.0, 45.0, ackermann.maxSteeringAngleDegProperty());
        HBox akMaxSpeedRow = boundSliderRow("Max speed:", 1.0, 20.0, ackermann.maxSpeedProperty());

        ackermann.wheelbaseProperty().addListener((o, ov, nv) -> {
            buildAckermannVisual();
            if (driveType == DriveType.ACKERMANN) repositionFpvCamera();
        });
        ackermann.trackWidthProperty().addListener((o, ov, nv) -> buildAckermannVisual());

        ackermannChassisBox = new VBox(6, akChassisLabel, wheelbaseRow, akTrackWidthRow, maxSteerRow, akMaxSpeedRow);
        ackermannChassisBox.setManaged(false);
        ackermannChassisBox.setVisible(false);

        driveTypeGroup.selectedToggleProperty().addListener((o, ov, nv) -> {
            boolean isAckermann = nv == ackermannRadio;
            switchDriveType(isAckermann ? DriveType.ACKERMANN : DriveType.DIFFERENTIAL);

            diffChassisBox.setVisible(!isAckermann);
            diffChassisBox.setManaged(!isAckermann);
            ackermannChassisBox.setVisible(isAckermann);
            ackermannChassisBox.setManaged(isAckermann);

            headerLabel.setText(isAckermann ? "Mobile Robot (Ackermann Steering)" : "Mobile Robot (Differential Drive)");
        });

        Label teleopLabel = new Label("Teleop (used when not following a path):");
        teleopLabel.setStyle("-fx-font-size: 11px; -fx-text-fill: #abb2bf; -fx-font-weight: bold;");

        teleopV = new Slider(-8, 8, 0);
        teleopOmega = new Slider(-3, 3, 0);
        HBox teleopVRow = sliderRow("v (units/s):", teleopV);
        HBox teleopOmegaRow = sliderRow("\u03C9 (rad/s):", teleopOmega);

        Button stopBtn = new Button("Stop / clear path");
        stopBtn.setMaxWidth(Double.MAX_VALUE);
        stopBtn.setStyle("-fx-background-color: #e06c75; -fx-text-fill: white; -fx-font-weight: bold;");
        stopBtn.setOnAction(e -> {
            clearPath();
            teleopV.setValue(0);
            teleopOmega.setValue(0);
        });

        Button cameraToggleBtn = new Button("Switch to 1st-person camera");
        cameraToggleBtn.setMaxWidth(Double.MAX_VALUE);
        cameraToggleBtn.setStyle("-fx-background-color: #c678dd; -fx-text-fill: white; -fx-font-weight: bold;");
        cameraToggleBtn.setOnAction(e -> {
            cameraMode = switch (cameraMode) {
                case ORBIT -> CameraMode.FPV;
                case FPV -> CameraMode.TOP_DOWN;
                case TOP_DOWN -> CameraMode.ORBIT;
            };

            Camera activeCamera = switch (cameraMode) {
                case ORBIT -> cameraRig.getCamera();
                case FPV -> fpvCamera;
                case TOP_DOWN -> topDownCamera;
            };
            subScene.setCamera(activeCamera);

            if (cameraMode == CameraMode.TOP_DOWN) {
                // Snap to the robot's current position immediately rather
                // than waiting for the next animation frame, so switching
                // views doesn't briefly show wherever the camera was last
                // parked (e.g. the origin, if the robot has since driven
                // away from it).
                Pose2D pose = activeModel().getPose();
                topDownCamera.setTranslateX(pose.x() * SCALE);
                topDownCamera.setTranslateZ(pose.y() * SCALE);
            }

            cameraToggleBtn.setText(switch (cameraMode) {
                case ORBIT -> "Switch to 1st-person camera";
                case FPV -> "Switch to top-down camera";
                case TOP_DOWN -> "Switch to orbit camera";
            });
        });

        Label hintLabel = new Label("Click the floor to queue a waypoint — following starts automatically, " +
                "and further clicks extend the path (pure-pursuit curvature control, not point-and-snap). " +
                "Switching drive type resets pose, trail, and path.");
        hintLabel.setWrapText(true);
        hintLabel.setStyle("-fx-font-size: 11px; -fx-text-fill: #abb2bf;");

        Button resetBtn = new Button("Reset pose to origin");
        resetBtn.setMaxWidth(Double.MAX_VALUE);
        resetBtn.setStyle("-fx-background-color: #3b3b4d; -fx-text-fill: #abb2bf; -fx-font-size: 11px;");
        resetBtn.setOnAction(e -> {
            activeModel().setPose(Pose2D.origin());
            trailGroup.getChildren().clear();
            clearPath();
        });

        Button selfTestBtn = new Button("Run kinematics self-test (both engines)");
        selfTestBtn.setMaxWidth(Double.MAX_VALUE);
        selfTestBtn.setStyle("-fx-background-color: #3b3b4d; -fx-text-fill: #abb2bf; -fx-font-size: 11px;");
        selfTestBtn.setOnAction(e -> runSelfTest());

        selfTestLabel = new Label();
        selfTestLabel.setWrapText(true);
        selfTestLabel.setMaxWidth(300);

        Label fileLabel = new Label("File I/O:");
        fileLabel.setStyle("-fx-font-size: 11px; -fx-text-fill: #abb2bf; -fx-font-weight: bold;");

        HBox fileBar = new HBox(8);
        Button exportBtn = new Button("Export JSON");
        exportBtn.setMaxWidth(Double.MAX_VALUE);
        HBox.setHgrow(exportBtn, Priority.ALWAYS);
        exportBtn.setStyle("-fx-background-color: #3b3b4d; -fx-text-fill: #98c379; -fx-border-color: #98c379; -fx-border-radius: 3;");
        exportBtn.setOnAction(e -> exportConfigToJson());

        Button importBtn = new Button("Import JSON");
        importBtn.setMaxWidth(Double.MAX_VALUE);
        HBox.setHgrow(importBtn, Priority.ALWAYS);
        importBtn.setStyle("-fx-background-color: #3b3b4d; -fx-text-fill: #c678dd; -fx-border-color: #c678dd; -fx-border-radius: 3;");
        importBtn.setOnAction(e -> importConfigFromJson());
        fileBar.getChildren().addAll(exportBtn, importBtn);

        ioStatusLabel = new Label();
        ioStatusLabel.setWrapText(true);
        ioStatusLabel.setMaxWidth(300);
        ioStatusLabel.setStyle("-fx-font-size: 11px;");

        hudLabel = new Label();
        hudLabel.setStyle("-fx-text-fill: #d0d0d0; -fx-font-family: monospace; -fx-font-size: 11px;");
        VBox hudBox = new VBox(hudLabel);
        hudBox.setStyle("-fx-background-color: #21252b; -fx-padding: 8; -fx-background-radius: 5;");

        panel.getChildren().addAll(
                headerLabel,
                driveTypeLabel, driveTypeRow,
                new Separator(),
                diffChassisBox, ackermannChassisBox,
                new Separator(),
                teleopLabel, teleopVRow, teleopOmegaRow, stopBtn, cameraToggleBtn, hintLabel,
                new Separator(),
                resetBtn, selfTestBtn, selfTestLabel,
                new Separator(),
                fileLabel, fileBar, ioStatusLabel,
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