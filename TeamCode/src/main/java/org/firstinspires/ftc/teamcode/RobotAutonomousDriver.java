package org.firstinspires.ftc.teamcode;

public class RobotAutonomousDriver {
    public final int TURN_DEGREE_TOLERANCE = 1;
    public final double DRIVE_POWER = 0.5;
    public final double TURN_POWER = 0.5;

    private final RobotHardware HARDWARE;

    public RobotAutonomousDriver(RobotHardware hardware) {
        HARDWARE = hardware;
    }

    public void drive(double inches)
    {
        int encoderTicks = (int)Math.round(
                inches / HARDWARE.WHEEL_CIRCUMFERENCE_IN * HARDWARE.TICKS_PER_MOTOR_REVOLUTION);

        HARDWARE.leftEncoderDrive(encoderTicks, DRIVE_POWER);
        HARDWARE.rightEncoderDrive(encoderTicks, DRIVE_POWER);
    }

    public void turn(int targetHeading) {
        Angle target = new Angle(targetHeading);

        int difference;

        while (true) {
            difference = target.difference(new Angle(HARDWARE.gyroHeading()));

            if (difference < -TURN_DEGREE_TOLERANCE) {
                HARDWARE.leftFreeDrive(-TURN_POWER);
                HARDWARE.rightFreeDrive(TURN_POWER);
            } else if (difference > TURN_DEGREE_TOLERANCE) {
                HARDWARE.leftFreeDrive(TURN_POWER);
                HARDWARE.rightFreeDrive(-TURN_POWER);
            } else {
                HARDWARE.leftFreeDrive(0);
                HARDWARE.rightFreeDrive(0);
                break;
            }
        }
    }

    private class Angle {
        public final int VALUE; //between 0 and 359 inclusive

        public Angle(int value) {
            VALUE = clamp(value);
        }

        public int difference(Angle other) {
            int multiplier = 1;
            int a = VALUE;
            int b = other.VALUE;

            if (b < a) {
                int temp = a;
                a = b;
                b = temp;
                multiplier = -1;
            }

            int counterclockwise = b - a;
            int clockwise = counterclockwise - 360;

            if (Math.abs(counterclockwise) < 180)
                return counterclockwise * multiplier;
            else
                return clockwise * multiplier;
        }

        private int clamp(int value) {
            return (value % 360 + 360) % 360;
        }
    }
}
