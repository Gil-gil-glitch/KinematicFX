package com.example.dhparameterfx;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/** Unit tests for {@link MobileRobotConfigIO}'s JSON serialization round-trip. */
class MobileRobotConfigIOTest {

    @Test
    void toJsonThenFromJson_roundTripsAllFields() {
        MobileRobotConfigIO.MobileRobotConfig original =
                new MobileRobotConfigIO.MobileRobotConfig(1.25, 4.5, 8.0, 3.3, -2.1, 137.5);

        String json = MobileRobotConfigIO.toJson(original);
        MobileRobotConfigIO.MobileRobotConfig parsed = MobileRobotConfigIO.fromJson(json);

        assertEquals(original.wheelRadius(), parsed.wheelRadius(), 1e-6);
        assertEquals(original.trackWidth(), parsed.trackWidth(), 1e-6);
        assertEquals(original.maxWheelSpeedRadPerSec(), parsed.maxWheelSpeedRadPerSec(), 1e-6);
        assertEquals(original.x(), parsed.x(), 1e-6);
        assertEquals(original.y(), parsed.y(), 1e-6);
        assertEquals(original.thetaDeg(), parsed.thetaDeg(), 1e-6);
    }

    @Test
    void toJson_handlesNegativeValues() {
        MobileRobotConfigIO.MobileRobotConfig config =
                new MobileRobotConfigIO.MobileRobotConfig(1.0, 3.0, 6.0, -5.0, -10.0, -90.0);

        String json = MobileRobotConfigIO.toJson(config);
        MobileRobotConfigIO.MobileRobotConfig parsed = MobileRobotConfigIO.fromJson(json);

        assertEquals(-5.0, parsed.x(), 1e-6);
        assertEquals(-10.0, parsed.y(), 1e-6);
        assertEquals(-90.0, parsed.thetaDeg(), 1e-6);
    }

    @Test
    void fromJson_isWhitespaceAndFieldOrderInsensitive() {
        String compact = "{\"maxWheelSpeedRadPerSec\":6,\"wheelRadius\":1,\"trackWidth\":3," +
                "\"pose\":{\"thetaDeg\":45,\"x\":1,\"y\":2}}";

        MobileRobotConfigIO.MobileRobotConfig parsed = MobileRobotConfigIO.fromJson(compact);

        assertEquals(1.0, parsed.wheelRadius(), 1e-6);
        assertEquals(3.0, parsed.trackWidth(), 1e-6);
        assertEquals(6.0, parsed.maxWheelSpeedRadPerSec(), 1e-6);
        assertEquals(1.0, parsed.x(), 1e-6);
        assertEquals(2.0, parsed.y(), 1e-6);
        assertEquals(45.0, parsed.thetaDeg(), 1e-6);
    }

    @Test
    void fromJson_rejectsNonObjectInput() {
        assertThrows(IllegalArgumentException.class, () -> MobileRobotConfigIO.fromJson("not json"));
        assertThrows(IllegalArgumentException.class, () -> MobileRobotConfigIO.fromJson(null));
    }

    @Test
    void fromJson_rejectsMissingField() {
        String missingTrackWidth = "{ \"wheelRadius\": 1.0, \"maxWheelSpeedRadPerSec\": 6.0, " +
                "\"pose\": { \"x\": 0, \"y\": 0, \"thetaDeg\": 0 } }";

        assertThrows(IllegalArgumentException.class, () -> MobileRobotConfigIO.fromJson(missingTrackWidth));
    }

    @Test
    void fromModelThenApplyTo_roundTripsThroughAModel() {
        MobileRobotModel source = new MobileRobotModel(1.5, 4.0, 9.0);
        source.setPose(new Pose2D(7.0, -3.0, 200.0));

        MobileRobotConfigIO.MobileRobotConfig config = MobileRobotConfigIO.fromModel(source);

        MobileRobotModel target = new MobileRobotModel(1.0, 1.0, 1.0);
        MobileRobotConfigIO.applyTo(target, config);

        assertEquals(source.getWheelRadius(), target.getWheelRadius(), 1e-6);
        assertEquals(source.getTrackWidth(), target.getTrackWidth(), 1e-6);
        assertEquals(source.getMaxWheelSpeedRadPerSec(), target.getMaxWheelSpeedRadPerSec(), 1e-6);
        assertEquals(source.getX(), target.getX(), 1e-6);
        assertEquals(source.getY(), target.getY(), 1e-6);
        assertEquals(source.getThetaDeg(), target.getThetaDeg(), 1e-6);
    }
}