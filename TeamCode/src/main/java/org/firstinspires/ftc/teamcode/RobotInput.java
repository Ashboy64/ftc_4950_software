package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class RobotInput {
    private final DoubleGamepad GAMEPAD;

    private static final boolean USE_DRIVE_INTERPOLATION = false;
    private static final double DRIVE_INTERPOLATION = 0.875;
    private static final boolean USE_ARM_INTERPOLATION = false;
    private static final double ARM_INTERPOLATION = 0.875;

    private final Interpolator LEFT_INTERPOLATOR = new Interpolator(DRIVE_INTERPOLATION);
    private final Interpolator RIGHT_INTERPOLATOR = new Interpolator(DRIVE_INTERPOLATION);
    private final Interpolator ARM_INTERPOLATOR = new Interpolator(ARM_INTERPOLATION);

    public RobotInput(Gamepad gamepad1, Gamepad gamepad2) {
        GAMEPAD = new DoubleGamepad(gamepad1, gamepad2);
    }

    public double getLeftPower() {
        double in;

        if (!GAMEPAD.x())
            in = GAMEPAD.leftStickY();
        else
            in = -GAMEPAD.rightStickY();

        if (USE_DRIVE_INTERPOLATION)
            return LEFT_INTERPOLATOR.value(in);
        else
            return in;
    }

    public double getRightPower() {
        double in;

        if (!GAMEPAD.x())
            in = GAMEPAD.rightStickY();
        else
            in = -GAMEPAD.leftStickY();

        if (USE_DRIVE_INTERPOLATION)
            return RIGHT_INTERPOLATOR.value(in);
        else
            return in;
    }

    public double getArmPower() {
        double in = GAMEPAD.rightTrigger() - GAMEPAD.leftTrigger();

        if (USE_ARM_INTERPOLATION)
            return ARM_INTERPOLATOR.value(in);
        else
            return in;
    }

    public double getClampPower() {
        double power = 0;

        if (GAMEPAD.leftBumper())
            power--;
        if (GAMEPAD.rightBumper())
            power++;

        return power;
    }

    private class Interpolator {
        private final double PER_SECOND;
        private double value;
        private long lastTime;
        private final double MIN;
        private final double MAX;

        public Interpolator(double perSecond) {
            this(perSecond, 0, -1, 1);
        }

        public Interpolator(double perSecond, double init, double min, double max) {
            PER_SECOND = perSecond;
            value = init;
            MIN = min;
            MAX = max;
        }

        public double value(double in) {
            long time = System.currentTimeMillis();
            double deltaTime = (time - lastTime) / 1000; //seconds elapsed since last update
            lastTime = time;

            value = value + (in - value) * PER_SECOND * deltaTime; //interpolates towards new value
            value = Math.max(MIN, Math.min(MAX, value)); //clamps within range

            return value;
        }
    }

    private class DoubleGamepad {
        private final Gamepad GAMEPAD1;
        private final Gamepad GAMEPAD2;

        public DoubleGamepad(Gamepad gamepad1, Gamepad gamepad2) {
            GAMEPAD1 = gamepad1;
            GAMEPAD2 = gamepad2;
        }

        public float leftStickX() {
            return priority(GAMEPAD1.left_stick_x, GAMEPAD2.left_stick_x);
        }

        public float rightStickX() {
            return priority(GAMEPAD1.right_stick_x, GAMEPAD2.right_stick_x);
        }

        public float leftStickY() {
            return priority(GAMEPAD1.left_stick_y, GAMEPAD2.left_stick_y);
        }

        public float rightStickY() {
            return priority(GAMEPAD1.right_stick_y, GAMEPAD2.right_stick_y);
        }

        public boolean leftStickButton() {
            return GAMEPAD1.left_stick_button || GAMEPAD2.left_stick_button;
        }

        public boolean rightStickButton() {
            return GAMEPAD1.right_stick_button || GAMEPAD2.right_stick_button;
        }

        public float leftTrigger() {
            return priority(GAMEPAD1.left_trigger, GAMEPAD2.left_trigger);
        }

        public float rightTrigger() {
            return priority(GAMEPAD1.right_trigger, GAMEPAD2.right_trigger);
        }

        public boolean leftBumper() {
            return GAMEPAD1.left_bumper || GAMEPAD2.left_bumper;
        }

        public boolean rightBumper() {
            return GAMEPAD1.right_bumper || GAMEPAD2.right_bumper;
        }

        public boolean a() {
            return GAMEPAD1.a || GAMEPAD2.a;
        }

        public boolean b() {
            return GAMEPAD1.b || GAMEPAD2.b;
        }

        public boolean x() {
            return GAMEPAD1.x || GAMEPAD2.x;
        }

        public boolean y() {
            return GAMEPAD1.y || GAMEPAD2.y;
        }

        public boolean dpadUp() {
            return GAMEPAD1.dpad_up || GAMEPAD2.dpad_up;
        }

        public boolean dpadDown() {
            return GAMEPAD1.dpad_down || GAMEPAD2.dpad_down;
        }

        public boolean dpadLeft() {
            return GAMEPAD1.dpad_left || GAMEPAD2.dpad_left;
        }

        public boolean dpadRight() {
            return GAMEPAD1.dpad_right || GAMEPAD2.dpad_right;
        }

        public boolean guide() {
            return GAMEPAD1.guide || GAMEPAD2.guide;
        }

        public boolean start() {
            return GAMEPAD1.start || GAMEPAD2.start;
        }

        public boolean back() {
            return GAMEPAD1.back || GAMEPAD2.back;
        }

        private float priority(float a, float b) {
            if (a != 0)
                return a;
            return b;
        }
    }
}