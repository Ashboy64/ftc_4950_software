package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.ControlUtils.*;

@TeleOp(name = "Teleop", group = "Concept")
public class Teleop extends OpMode {
    private enum DriveMode {
        FORWARD(1, 0.5, 3, 1.5),
        BACKWARD(0.75, 0.75, 2, 2),
        TURN(0.75, 0.75, 4, 4);

        private final double POWER_DOWN;
        private final double POWER_UP;
        private final double ACCEL_DOWN;
        public final double ACCEL_UP;

        DriveMode(double powerDown, double powerUp, double accelDown, double accelUp) {
            POWER_DOWN = powerDown;
            POWER_UP = powerUp;
            ACCEL_DOWN = accelDown;
            ACCEL_UP = accelUp;
        }

        private double power(double armHeight) {
            return lerp(POWER_DOWN, POWER_UP, armHeight);
        }

        private double accel(double armHeight) {
            return lerp(ACCEL_DOWN, ACCEL_UP, armHeight);
        }

        public static double power(double armHeight, double left, double right)
        {
            return mode(left, right).power(armHeight);
        }

        public static double accel(double armHeight, double left, double right)
        {
            return mode(left, right).accel(armHeight);
        }

        public static DriveMode mode(double left, double right) {
            if (left > 0 && right > 0) {
                return FORWARD;
            } else if (left < 0 && right < 0) {
                return BACKWARD;
            } else {
                return TURN;
            }
        }
    }

    private static final boolean TOUCH_LIMIT_ARM = true;

    private static final double ARM_MIN_POWER_FALLING = 0.25;
    private static final double ARM_MIN_POWER_LIFTING = 0.5;
    private static final double ARM_BALANCE_ANGLE_OFFSET = 45;

    private RobotInput INPUT;
    private RobotHardware HARDWARE;

    private final Interpolator LEFT_INTERPOLATOR = new Interpolator(DriveMode.FORWARD.power(0));
    private final Interpolator RIGHT_INTERPOLATOR = new Interpolator(DriveMode.FORWARD.power(0));

    @Override
    public void init() {
        INPUT = new RobotInput(gamepad1, gamepad2);
        HARDWARE = new RobotHardware(hardwareMap);
        HARDWARE.motorZeroPowerBrake(false);
    }

    @Override
    public void loop() {
        updateClamp();
        updateJewel();

        double armAngle = (HARDWARE.armPosition() - ARM_BALANCE_ANGLE_OFFSET + 360) % 360;

        double armIn = INPUT.getArmPower();
        double armPower = armPower(armIn, armAngle);
        HARDWARE.armDrive(armPower);
        double armHeight = armHeight(armAngle);

        telemetry.addLine(String.format(Locale.ENGLISH, "arm angle: %.4f, sin: %.4f, cos: %.4f, power: %.4f, height: %.4f",
                armAngle, sin(armAngle), cos(armAngle), armPower, armHeight));

        double leftIn = INPUT.getLeftPower();
        double rightIn = INPUT.getRightPower();

        double power = DriveMode.power(armHeight, leftIn, rightIn);
        double accel = DriveMode.accel(armHeight, leftIn, rightIn);

        LEFT_INTERPOLATOR.setMaxDelta(accel);
        RIGHT_INTERPOLATOR.setMaxDelta(accel);

        double left = LEFT_INTERPOLATOR.value(leftIn * power);
        double right = RIGHT_INTERPOLATOR.value(rightIn * power);

        HARDWARE.freeDrive(left, right);
        telemetry.addLine(String.format(Locale.ENGLISH, "drive direction: %s, power: %.4f and %.4f (%.4f), accel: %.4f",
                DriveMode.mode(leftIn, rightIn).toString(), left, right, power, accel));
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

    private void updateJewel() {
        if (INPUT.GAMEPAD.x()) {
            HARDWARE.setJewelArmPosition(0);
        } else {
            HARDWARE.setJewelArmPosition(1);
        }
    }

    private double armPower(double armPower, double armAngle) {
        if (armPower == 0) {
            return 0;
        }

        double x = cos(armAngle);

        boolean lifting = x > 0 == armPower > 0;
        telemetry.addLine("arm lifting: " + lifting);

        if (lifting) {
            return armPower * lerp(ARM_MIN_POWER_LIFTING, 1, Math.abs(x));
        } else {
            return armPower * ARM_MIN_POWER_FALLING;
        }
    }

    private double armHeight(double armAngle) {
        return (sin(armAngle) + 1) / 2;
    }
}