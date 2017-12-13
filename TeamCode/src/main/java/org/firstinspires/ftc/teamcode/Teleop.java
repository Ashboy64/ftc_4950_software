package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop", group = "Concept")
public class Teleop extends OpMode {
    private static final boolean TOUCH_LIMIT_ARM = true;

    private static final double DRIVE_TURN_DOWN_POWER = .75; //0.75;
    private static final double DRIVE_TURN_UP_POWER = .75; //0.5;
    private static final double DRIVE_TURN_DOWN_ACCEL = 8;
    private static final double DRIVE_TURN_UP_ACCEL = 4;

    private static final double DRIVE_FORWARD_DOWN_POWER = 1;
    private static final double DRIVE_BACKWARD_DOWN_POWER = 0.75;
    private static final double DRIVE_FORWARD_DOWN_ACCEL = 4;
    private static final double DRIVE_BACKWARD_DOWN_ACCEL = 1.5;

    private static final double DRIVE_FORWARD_UP_POWER = 0.5;
    private static final double DRIVE_BACKWARD_UP_POWER = 0.75;
    private static final double DRIVE_FORWARD_UP_ACCEL = 1;
    private static final double DRIVE_BACKWARD_UP_ACCEL = 1.5;

    private static final double ARM_MIN_POWER = 0.375;
    private static final double ARM_BALANCE_ANGLE_OFFSET = 45;

    private RobotInput INPUT;
    private RobotHardware HARDWARE;

    private final Interpolator LEFT_INTERPOLATOR = new Interpolator(DRIVE_TURN_DOWN_ACCEL);
    private final Interpolator RIGHT_INTERPOLATOR = new Interpolator(DRIVE_TURN_DOWN_ACCEL);

    @Override
    public void init() {
        INPUT = new RobotInput(gamepad1, gamepad2);
        HARDWARE = new RobotHardware(hardwareMap);
    }

    @Override
    public void loop() {
        updateClamp();

        double armAngle = (HARDWARE.armPosition() - ARM_BALANCE_ANGLE_OFFSET + 360) % 360;
        double armIn = INPUT.getArmPower();
        double armPower = armPower(armIn, armAngle);
        HARDWARE.armDrive(armPower);

        telemetry.addLine(String.format("arm angle: %.4f, sin: %.4f, cos: %.4f, power: %.4f",
                armAngle, sin(armAngle), cos(armAngle), armPower));

        double leftIn = INPUT.getLeftPower();
        double rightIn = INPUT.getRightPower();

        double power = drivePower(leftIn, rightIn, armAngle);
        double accel = driveAccel(leftIn, rightIn, armAngle);

        LEFT_INTERPOLATOR.setMaxDelta(accel);
        RIGHT_INTERPOLATOR.setMaxDelta(accel);

        double left = LEFT_INTERPOLATOR.value(leftIn * power);
        double right = RIGHT_INTERPOLATOR.value(rightIn * power);

        HARDWARE.freeDrive(left, right);
        telemetry.addLine(String.format("drive power: %.4f, accel: %.4f", power, accel));
    }

    private void updateClamp() {
        double clampPower = INPUT.getClampPower();
        if (TOUCH_LIMIT_ARM) {
            if (HARDWARE.getTouchOpen()) {
                clampPower = Math.max(clampPower, 0);
            } else if (HARDWARE.getTouchClosed()) {
                clampPower = Math.min(clampPower, 0);
            }
        }
        HARDWARE.setClampPower(clampPower);
    }

    private double armPower(double armPower, double armAngle) {
        if (armPower == 0) {
            return 0;
        }

        double x = cos(armAngle);

        boolean lifting = x > 0 == armPower > 0;
        telemetry.addLine("arm lifting: " + lifting);

        if (lifting) {
            return clamp(armPower * (Math.max(Math.abs(x), ARM_MIN_POWER)));
        } else {
            return armPower * ARM_MIN_POWER;
        }
    }

    private double drivePower(double left, double right, double armAngle) {
        if (left > 0 && right > 0) {
            telemetry.addLine("forward");
            return forwardPower(armAngle);
        } else if (left < 0 && right < 0) {
            telemetry.addLine("backward");
            return backwardPower(armAngle);
        } else {
            telemetry.addLine("turning/stationary");
            return turnPower(armAngle);
        }
    }

    private double driveAccel(double left, double right, double armAngle) {
        if (left < 0 && right < 0) {
            return forwardAccel(armAngle);
        } else if (left > 0 && right > 0) {
            return backwardAccel(armAngle);
        } else {
            return turnAccel(armAngle);
        }
    }

    private double turnPower(double armAngle) {
        return lerp(DRIVE_TURN_DOWN_POWER, DRIVE_TURN_UP_POWER, armHeight(armAngle));
    }

    private double turnAccel(double armAngle) {
        return lerp(DRIVE_TURN_DOWN_ACCEL, DRIVE_TURN_UP_ACCEL, armHeight(armAngle));
    }

    private double forwardPower(double armAngle) {
        return lerp(DRIVE_FORWARD_DOWN_POWER, DRIVE_FORWARD_UP_POWER, armHeight(armAngle));
    }

    private double forwardAccel(double armAngle) {
        return lerp(DRIVE_FORWARD_DOWN_ACCEL, DRIVE_FORWARD_UP_ACCEL, armHeight(armAngle));
    }

    private double backwardPower(double armAngle) {
        return lerp(DRIVE_BACKWARD_DOWN_POWER, DRIVE_BACKWARD_UP_POWER, armHeight(armAngle));
    }

    private double backwardAccel(double armAngle) {
        return lerp(DRIVE_BACKWARD_DOWN_ACCEL, DRIVE_BACKWARD_UP_ACCEL, armHeight(armAngle));
    }

    private double armHeight(double armAngle) {
        //we want arm angles within a 30-degree range of minimum and maximum to count as min/max
        double clampRange = sin(60);
        return clamp(-clampRange, clampRange, (sin(armAngle) + 1) / 2) / clampRange;
    }

    private double sin(double degrees) {
        return Math.sin(Math.toRadians(degrees));
    }

    private double cos(double degrees) {
        return Math.cos(Math.toRadians(degrees));
    }

    private double clamp(double d, double min, double max) {
        return Math.min(max, Math.max(min, d));
    }

    private double clamp(double d) {
        return clamp(d, -1, 1);
    }

    private double lerp(double a, double b, double t) {
        return (b - a) * t + a;
    }

    private class Interpolator {
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
            value = Math.max(MIN, Math.min(MAX, value));

            return value;
        }
    }
}