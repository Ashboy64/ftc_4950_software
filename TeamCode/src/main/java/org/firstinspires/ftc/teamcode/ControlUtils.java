package org.firstinspires.ftc.teamcode;

public class ControlUtils {
    public static double lerp(double min, double max, double t) {
        return clamp((max - min) * t + min, min, max);
    }

    public static double invLerp(double d, double min, double max) {
        return (d - min) / (max - min);
    }

    public static double sin(double degrees) {
        return Math.sin(Math.toRadians(degrees));
    }

    public static double cos(double degrees) {
        return Math.cos(Math.toRadians(degrees));
    }

    public static double clamp(double d, double min, double max) {
        return Math.min(max, Math.max(min, d));
    }

    public static double clamp(double d) {
        return clamp(d, -1, 1);
    }

    public static double scale(double in, double oldMin, double oldMax, double newMin, double newMax) {
        return lerp(newMin, newMax, invLerp(in, oldMin, oldMax));
    }
}

class Interpolator {
    private double maxDelta;
    private double value;
    private long lastTime;
    private final double MIN;
    private final double MAX;

    public Interpolator(double perSecond) {
        this(perSecond, 0, -1, 1);
    }

    public Interpolator(double perSecond, double init, double min, double max) {
        maxDelta = perSecond;
        value = init;
        MIN = min;
        MAX = max;
        lastTime = System.currentTimeMillis();
    }

    public void setMaxDelta(double delta) {
        this.maxDelta = delta;
    }

    public double value(double in) {
        long time = System.currentTimeMillis();

        //seconds elapsed since last update
        double deltaTime = (time - lastTime) / 1000.0;
        lastTime = time;

        double change = in - value;
        change = Math.max(-maxDelta * deltaTime, Math.min(maxDelta * deltaTime, change));

        //interpolates towards new value
        value += change;

        //clamps within range
        value = ControlUtils.clamp(value, MIN, MAX);

        return value;
    }
}