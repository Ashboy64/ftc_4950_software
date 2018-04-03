package org.firstinspires.ftc.teamcode;

/**
 * A collection of general utility methods and helper classes. All angles are
 * in radians unless otherwise specified.
 */
public class Utils {
    /**
     * Linearly interpolates between two values.
     *
     * @param a the start value
     * @param b the end value
     * @param t the interpolation value, with 0 corresponding to a and 1 corresponding to b
     * @return the value between a and b interpolated by t, clamped between a and b
     */
    public static double lerp(double a, double b, double t) {
        return clamp((b - a) * t + a, Math.min(a, b), Math.max(a, b));
    }

    /**
     * Returns the average of two values.
     *
     * @param a the first value
     * @param b the second value
     * @return the average
     */
    public static double average(double a, double b) {
        return lerp(a, b, 0.5);
    }

    /**
     * Returns the numerical distance between two values (the absolute value of
     * the difference).
     *
     * @param a
     * @param b
     * @return
     */
    public static double distance(double a, double b) {
        return Math.abs(b - a);
    }

    /**
     * Clamps a value within a range.
     *
     * @param d the initial value
     * @param a one endpoint of the acceptable range
     * @param b the other endpoint of the acceptable range
     * @return d clamped between a and b
     */
    public static double clamp(double d, double a, double b) {
        return Math.min(Math.max(a, b), Math.max(Math.min(a, b), d));
    }

    /**
     * Calculates the difference between two angles, normalized between -π and π.
     *
     * @param a the first angle
     * @param b the second angle
     * @return the difference between the two angles
     */
    public static double angleDifference(double a, double b) {
        double angle = angleNormalize(a - b);
        if (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Normalizes an angle (returns its corresponding coterminal angle >= 0 and < 2π)
     *
     * @param angle the angle to normalize
     * @return the normalized angle
     */
    public static double angleNormalize(double angle) {
        angle %= 2 * Math.PI;
        if (angle < 0) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Converts an angle measure from the unit circle representation to a heading.
     *
     * @param unitCircle the angle measure in radians on the unit circle
     * @return the angle measure converted to a heading, also in radians
     */
    public static double toHeading(double unitCircle) {
        return angleNormalize((Math.PI / 2) - unitCircle);
    }

    /**
     * Converts an angle measure from a heading to the unit circle representation.
     *
     * @param heading the heading in radians
     * @return the heading converted to a unit circle angle measure, also in radians
     */
    public static double toUnitCircle(double heading) {
        return (Math.PI / 2) - heading;
    }
}

/**
 * Represents a point in 2D space.
 */
class Point {
    public final double X;
    public final double Y;
    public static final Point ZERO = new Point(0, 0);

    public Point(double x, double y) {
        X = x;
        Y = y;
    }

    public Vector vectorTo(Point other) {
        return new Vector(other.X - X, other.Y - Y);
    }

    /**
     * Returns the square of the distance between two Points. Useful when only
     * comparing distances to each other, saving a square root operation.
     *
     * @param a the first Point
     * @param b the second Point
     * @return the square of the distance between the Points
     */
    public static double distanceSquared(Point a, Point b) {
        return (b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y);
    }

    /**
     * Returns the distance between two Points.
     *
     * @param a the first Point
     * @param b the second Point
     * @return the distance between the Points
     */
    public static double distance(Point a, Point b) {
        return Math.sqrt(distanceSquared(a, b));
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f)", X, Y);
    }
}

/**
 * Represents a vector in 2D space.
 */
class Vector extends Point {
    public Vector(double x, double y) {
        super(x, y);
    }

    public double magnitude() {
        return distance(ZERO, this);
    }

    public double getAngle() {
        if (Y == 0) {
            return X < 0 ? Math.PI : 0;
        } else if (X == 0) {
            return Y < 0 ? (3 * Math.PI / 2) : (Math.PI / 2);
        } else {
            double angle = Math.atan(Y / X);
            if (X < 0) {
                angle += Math.PI;
            }
            return Utils.angleNormalize(angle);
        }
    }
}