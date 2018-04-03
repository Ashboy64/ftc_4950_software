package org.firstinspires.ftc.teamcode;

/**
 * Represents a path from the origin to an endpoint, defined parametrically.
 */
public abstract class ParametricPath {
    abstract double getX(double t);

    abstract double getY(double t);

    /**
     * Returns the Point corresponding to the given distance along the curve.
     *
     * @param t the distance along the curve
     * @return the Point corresponding to the given value of t
     */
    public Point getPoint(double t) {
        return new Point(getX(t), getY(t));
    }

    /**
     * Returns the value of t corresponding to the point on the curve closest to the point given.
     *
     * @param point the current position
     * @return the closest value of t
     */
    public abstract double nearestT(Point point);

    /**
     * Returns the total getLength of the curve.
     *
     * @return the getLength of the curve
     */
    public abstract double getLength();
}

/**
 * Degenerate path, might still be useful for point targeting
 */
class Dot extends ParametricPath {
    private final int X;
    private final int Y;

    public Dot(int x, int y) {
        X = x;
        Y = y;
    }

    @Override
    public double nearestT(Point point) {
        return 0.5;
    }

    @Override
    double getX(double t) {
        return X;
    }

    @Override
    double getY(double t) {
        return Y;
    }

    @Override
    public double getLength() {
        return 1;
    }
}

/**
 * A line segment between two points
 */
class Line extends ParametricPath {
    private final Point START;
    private final Point END;
    private final double LENGTH;

    /**
     * Creates a Line that begins at one point and terminates at another.     *
     * @param start the start point
     * @param end the end point
     */
    public Line(Point start, Point end) {
        START = start;
        END = end;
        LENGTH = Point.distance(start, end);
    }

    @Override
    public double getX(double t) {
        return Utils.lerp(START.X, END.X, t / getLength());
    }

    @Override
    public double getY(double t) {
        return Utils.lerp(START.Y, END.Y, t / getLength());
    }

    @Override
    public double nearestT(Point point) {
        //terrible binary-search-ish solution instead of actual math
        //apologies to Ishika

        double t = 0;
        double lastDist = Point.distance(point, getPoint(t));
        double hop = 0.5;

        while (hop > 0.00001) {
            t += hop;
            double dist = Point.distance(point, getPoint(t));

            if (dist > lastDist) {
                t -= hop;
                hop *= -0.5;
            }

            lastDist = dist;
        }

        return t;
    }

    @Override
    public double getLength() {
        return LENGTH;
    }
}

/**
 * The unit circle
 */
/*
class Circle extends ParametricPath {
    @Override
    public double getX(double t) {
        return Math.cos(t);
    }

    @Override
    public double getY(double t) {
        return Math.cos(t);
    }

    @Override
    public double nearestT(Point point) {
        if (y == 0) {
            return x < 0 ? Math.PI : 0;
        } else if (x == 0) {
            return y < 0 ? (3 * Math.PI / 2) : (Math.PI / 2);
        } else {
            double angle = Math.atan(y / x);
            if (x < 0) {
                angle += Math.PI;
            }
            return Utils.angleNormalize(angle);
        }
    }

    @Override
    public double minT() {
        return 0;
    }

    @Override
    public double maxT() {
        return 2 * Math.PI;
    }
}
*/