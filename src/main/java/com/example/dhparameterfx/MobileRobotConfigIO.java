package com.example.dhparameterfx;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Reads/writes a mobile robot's chassis geometry + pose as JSON text.
 * <p>
 * This intentionally mirrors {@code kinematic3DApp.exportToJson}/
 * {@code importFromJson}: a hand-built JSON object and regex-based field
 * extraction, rather than pulling in a JSON library, so the two persistence
 * paths (arm DH table vs. mobile robot config) stay consistent in style.
 * Unlike those methods, the JSON building/parsing here is factored out from
 * any {@code Stage}/{@code FileChooser} code so it can be unit tested
 * without a JavaFX toolkit running.
 */
public class MobileRobotConfigIO {

    /** Everything needed to fully restore a {@link MobileRobotModel}'s configuration and pose. */
    public record MobileRobotConfig(
            double wheelRadius,
            double trackWidth,
            double maxWheelSpeedRadPerSec,
            double x,
            double y,
            double thetaDeg) {
    }

    public static MobileRobotConfig fromModel(MobileRobotModel model) {
        return new MobileRobotConfig(
                model.getWheelRadius(),
                model.getTrackWidth(),
                model.getMaxWheelSpeedRadPerSec(),
                model.getX(),
                model.getY(),
                model.getThetaDeg());
    }

    public static void applyTo(MobileRobotModel model, MobileRobotConfig config) {
        model.wheelRadiusProperty().set(config.wheelRadius());
        model.trackWidthProperty().set(config.trackWidth());
        model.maxWheelSpeedRadPerSecProperty().set(config.maxWheelSpeedRadPerSec());
        model.setPose(new Pose2D(config.x(), config.y(), config.thetaDeg()));
    }

    /**
     * Serializes to a single JSON object, e.g.:
     * {@code { "type": "differential_drive", "wheelRadius": 1.0000, "trackWidth": 3.0000,
     *   "maxWheelSpeedRadPerSec": 6.0000, "pose": { "x": 0.0000, "y": 0.0000, "thetaDeg": 0.0000 } }}
     */
    public static String toJson(MobileRobotConfig config) {
        return String.format(
                "{\n" +
                        "  \"type\": \"differential_drive\",\n" +
                        "  \"wheelRadius\": %.4f,\n" +
                        "  \"trackWidth\": %.4f,\n" +
                        "  \"maxWheelSpeedRadPerSec\": %.4f,\n" +
                        "  \"pose\": { \"x\": %.4f, \"y\": %.4f, \"thetaDeg\": %.4f }\n" +
                        "}",
                config.wheelRadius(), config.trackWidth(), config.maxWheelSpeedRadPerSec(),
                config.x(), config.y(), config.thetaDeg());
    }

    /**
     * Parses JSON produced by {@link #toJson}. Field order and whitespace
     * don't matter (regex-based, like {@code kinematic3DApp.extractJsonDouble}),
     * but all six numeric fields must be present.
     *
     * @throws IllegalArgumentException if the JSON is malformed or missing required fields
     */
    public static MobileRobotConfig fromJson(String json) {
        if (json == null || !json.trim().startsWith("{")) {
            throw new IllegalArgumentException("Invalid JSON: expected an object.");
        }

        double wheelRadius = requireDouble(json, "wheelRadius");
        double trackWidth = requireDouble(json, "trackWidth");
        double maxWheelSpeed = requireDouble(json, "maxWheelSpeedRadPerSec");
        double x = requireDouble(json, "x");
        double y = requireDouble(json, "y");
        double thetaDeg = requireDouble(json, "thetaDeg");

        return new MobileRobotConfig(wheelRadius, trackWidth, maxWheelSpeed, x, y, thetaDeg);
    }

    private static double requireDouble(String json, String key) {
        Pattern pattern = Pattern.compile("\"" + key + "\"\\s*:\\s*([-+]?[0-9]*\\.?[0-9]+)");
        Matcher matcher = pattern.matcher(json);
        if (!matcher.find()) {
            throw new IllegalArgumentException("Missing or malformed field: \"" + key + "\"");
        }
        return Double.parseDouble(matcher.group(1));
    }
}