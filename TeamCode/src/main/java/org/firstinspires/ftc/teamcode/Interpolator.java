package org.firstinspires.ftc.teamcode;

/**
 * A time-based value interpolator.
 */
public class Interpolator {
    //the maximum allowed change in value, per second
    private double maxDelta;

    //the current value
    private double value;

    //the last time that the value was interpolated, stored to compute change in time
    private long lastTime;

    //minimum and maximum values
    private final double MIN;
    private final double MAX;

    /**
     * Creates a default Interpolator that interpolates a value between -1 and 1.
     * @param maxChangePerSecond the maximum allowed change in value, per second
     */
    public Interpolator(double maxChangePerSecond) {
        this(maxChangePerSecond, 0, -1, 1);
    }

    /**
     * Creates a new Interpolator.
     * @param maxChangePerSecond the maximum allowed change in value, per second
     * @param init the initial value
     * @param min the minimum allowed value
     * @param max the maximum allowed value
     */
    public Interpolator(double maxChangePerSecond, double init, double min, double max) {
        maxDelta = maxChangePerSecond;
        value = init;
        MIN = min;
        MAX = max;
        lastTime = System.currentTimeMillis();
    }

    public void setMaxDelta(double delta) {
        this.maxDelta = delta;
    }

    /**
     * Gets the current value.
     * @return the current value
     */
    public double value()
    {
        return value;
    }

    /**
     * Interpolates towards a target value and returns the new current value.
     * @param target the target value
     * @return the new current value
     */
    public double value(double target) {
        //calculate seconds elapsed since last update
        long time = System.currentTimeMillis();
        double deltaTime = (time - lastTime) / 1000.0;
        lastTime = time;

        //calculates maximum allowed change
        double change = target - value;
        change = Utils.clamp(change, -maxDelta * deltaTime, maxDelta * deltaTime);

        //interpolates towards new value, and clamps within range
        value += change;
        value = Utils.clamp(value, MIN, MAX);

        return value;
    }
}
