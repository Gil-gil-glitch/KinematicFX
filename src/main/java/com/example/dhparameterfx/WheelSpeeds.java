package com.example.dhparameterfx;

/**
 * Angular wheel speeds for a two-wheel differential drive base, in rad/s.
 * Positive values spin the wheel "forward" (contact point moving in the
 * robot's +x body direction).
 */
public record WheelSpeeds(double leftRadPerSec, double rightRadPerSec) {

    public static WheelSpeeds zero() {
        return new WheelSpeeds(0.0, 0.0);
    }

    /** Clamp both wheels to +/- maxRadPerSec, preserving the turn ratio only if possible. */
    public WheelSpeeds clamped(double maxRadPerSec) {
        double l = Math.max(-maxRadPerSec, Math.min(maxRadPerSec, leftRadPerSec));
        double r = Math.max(-maxRadPerSec, Math.min(maxRadPerSec, rightRadPerSec));
        return new WheelSpeeds(l, r);
    }
}