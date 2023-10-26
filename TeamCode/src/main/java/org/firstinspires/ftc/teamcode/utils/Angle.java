package org.firstinspires.ftc.teamcode.utils;

public final class Angle {

    public static final double halfPI = Math.PI / 2;
    public static final double pi = Math.PI;
    public static final double twoPI = Math.PI * 2;

    public static double wrapAngle0_2PI(final double theta) {
        return (theta % twoPI + twoPI) % twoPI;
    }
    // convert 0-360 iRadians

    public static double wrapAnglePlusMinusPI(final double theta) {
        final double wrapped = theta % twoPI; // Getting the angle smallest form (not exceeding a
        // full turn).
// convert -180-180 in Radians
        // Adding or subtracting two pi to fit the range of +/- pi.
        if (wrapped > pi) {
            return wrapped - twoPI;
        } else if (wrapped < -pi) {
            return wrapped + twoPI;
        } else {
            return wrapped;
        }
    }

    // Functions to convert between degrees and radians:
    public static double degToRad(final double theta) {
        return Math.toRadians(theta);
    } //double

    public static double radToDeg(final double theta) {
        return Math.toDegrees(theta);
    } //double

    public static double projectBetweenPlanes(final double theta, final double alpha) {
        if (alpha < 0) {
            return Math.atan(Math.tan(theta) / Math.cos(alpha));
        } else {
            return Math.atan(Math.tan(theta) * Math.cos(alpha));
        }
    }
    // alpha is the angle between the planes.
    // For more information check:
    // https://math.stackexchange.com/questions/2207665/projecting-an-angle-from-one-plane-to-another-plane
}