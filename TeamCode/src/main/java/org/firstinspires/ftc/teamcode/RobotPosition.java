package org.firstinspires.ftc.teamcode;

/**
 * Calculates the current robot position using the distances traveled by the
 * left and right wheels.
 */
public class RobotPosition {
    //distance between left and right wheels
    private final double WIDTH;

    //x and y position of robot
    private double x;
    private double y;

    //robot angle on the unit circle
    private double angle;

    //last recorded encoder values for motors
    private int lastLeftTicks;
    private int lastRightTicks;

    public RobotPosition() {
        WIDTH = Robot.DRIVE_WIDTH;
        reset();
        lastLeftTicks = Robot.getRobot().DRIVE_LEFT.getCurrentPosition();
        lastRightTicks = Robot.getRobot().DRIVE_RIGHT.getCurrentPosition();
    }

    /**
     * Resets the local coordinate system, so that the current position and
     * rotation are treated as the new origin and initial rotation.
     */
    public void reset() {
        x = 0;
        y = 0;
        angle = Math.PI / 2;
    }

    /**
     * Recalculates the current robot position based on the change in encoder
     * values.
     */
    public void update() {
        int leftTicks = Robot.getRobot().DRIVE_LEFT.getCurrentPosition();
        int rightTicks = Robot.getRobot().DRIVE_RIGHT.getCurrentPosition();

        double leftInches = Robot.encoderTicksToInches(leftTicks - lastLeftTicks);
        double rightInches = Robot.encoderTicksToInches(rightTicks - lastRightTicks);

        move(leftInches, rightInches);

        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
    }

    //recalculates robot position based on distance driven by left and right wheels
    private void move(double leftInches, double rightInches) {
        leftInches += (0.0000001 * (Math.random() - 0.5));
        rightInches += (0.0000001 * (Math.random() - 0.5));

        double arcRadius = (WIDTH / 2) * (rightInches + leftInches) / (rightInches - leftInches);
        double arcLength = (leftInches + rightInches) / 2;
        double arcAngle = arcLength / arcRadius;

        double translateAngle = arcAngle / 2;
        double translateDistance = Math.sqrt(2 * arcRadius * arcRadius * (1 - Math.cos(arcAngle)));

        x += translateDistance * Math.cos(angle + translateAngle);
        y += translateDistance * Math.sin(angle + translateAngle);
        angle += arcAngle;
    }

    /**
     * Gets the current position of the robot relative to the initial
     * position, where the robot starts at the origin facing positive y.
     * @return the robot position
     */
    public Point getPosition() {
        return new Point(x, y);
    }

    /**
     * Returns the angle that the robot is facing as a unit circle measure, in
     * radians.
     * @return the robot rotation angle on the unit circle
     */
    public double getAngle() {
        return Utils.angleNormalize(angle);
    }

    @Override
    public String toString() {
        return String.format("%s heading %.2f", getPosition(), Math.toDegrees(Utils.toHeading(angle)));
    }
}
