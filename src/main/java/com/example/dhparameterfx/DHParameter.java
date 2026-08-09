package com.example.dhparameterfx;

public record DHParameter(double a, double alpha, double d, double theta) {

    public Matrix4x4 toTransformMatrix() {

        double radTheta = Math.toRadians(theta); // Handles Angles in Radians now, instead of degrees
        double radAlpha = Math.toRadians(alpha);

        // parameters
        double cosT = Math.cos(radTheta);
        double sinT = Math.sin(radTheta);
        double cosA = Math.cos(radAlpha);
        double sinA = Math.sin(radAlpha);

        return new Matrix4x4(new double[][]{
                { cosT, -sinT * cosA,  sinT * sinA, a * cosT },
                { sinT,  cosT * cosA, -cosT * sinA, a * sinT },
                {     0,        sinA,         cosA,        d },
                {     0,           0,            0,        1 }
        });
    }
}