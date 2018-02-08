package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.ControlUtils.*;

@TeleOp(name = "Teleop", group = "Concept")
public class Teleop extends OpMode {
    private enum DriveMode {
        FORWARD(1, 0.5, 3, 1),
        BACKWARD(0.75, 0.75, 2, 2),
        TURN(0.75, 0.75, 4, 3);

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

    private static final double ARM_MIN_POWER_FALLING = 0.375;
    private static final double ARM_MIN_POWER_LIFTING = 0.5;
    private static final double ARM_BALANCE_ANGLE_OFFSET = 45;

    private RobotInput INPUT;

    public static final double TICKS_PER_MOTOR_REVOLUTION = 1120;
    public static final double ARM_SPROCKET_RATIO = 15.0 / 54;

    private CRServo clampServo;

    private DcMotor armMotor;
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private DigitalChannel armTouchOpen;

    private Servo jewelServo;

    private final Interpolator LEFT_INTERPOLATOR = new Interpolator(DriveMode.FORWARD.power(0));
    private final Interpolator RIGHT_INTERPOLATOR = new Interpolator(DriveMode.FORWARD.power(0));

    @Override
    public void init() {
        INPUT = new RobotInput(gamepad1, gamepad2);
        initHardware();
    }

    private void initHardware() {
        clampServo = hardwareMap.crservo.get("clampServo");

        armTouchOpen = hardwareMap.get(DigitalChannel.class, "tsOpen");
        armTouchOpen.setMode(DigitalChannel.Mode.INPUT);

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        armMotor = hardwareMap.dcMotor.get("armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetArm();

        jewelServo = hardwareMap.servo.get("jewelServo");
    }

    @Override
    public void loop() {
        updateClamp();
        updateJewel();

        double armAngle = (armPosition() - ARM_BALANCE_ANGLE_OFFSET + 360) % 360;

        double armIn = INPUT.getArmPower();
        double armPower = armPower(armIn, armAngle);
        armDrive(armPower);
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

        freeDrive(left, right);
        telemetry.addLine(String.format(Locale.ENGLISH, "drive direction: %s, power: %.4f and %.4f (%.4f), accel: %.4f",
                DriveMode.mode(leftIn, rightIn).toString(), left, right, power, accel));
    }

    private void updateClamp() {
        double clampPower = INPUT.getClampPower();
        if (TOUCH_LIMIT_ARM) {
            if (getTouchOpen()) {
                clampPower = Math.max(clampPower, 0);
            }
        }
        setClampPower(clampPower);
    }

    private void updateJewel() {
        if (INPUT.GAMEPAD.x()) {
            setJewelArmPosition(0);
        } else {
            setJewelArmPosition(1);
        }
    }

    private double armPower(double armPower, double armAngle) {
        if (INPUT.GAMEPAD.a())
        {
            telemetry.addLine("arm power override");
            return armPower;
        }

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

    private void resetArm() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void armDrive(double power) {
        armMotor.setPower(power);
    }

    private void freeDrive(double left, double right) {
        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }

    private void setClampPower(double power) {
        clampServo.setPower(power);
    }

    private boolean getTouchOpen() {
        return !armTouchOpen.getState();
    }

    public double armPosition() {
        return armMotor.getCurrentPosition()
                / TICKS_PER_MOTOR_REVOLUTION
                * ARM_SPROCKET_RATIO
                * 360;
    }

    private void setJewelArmPosition(double position) {
        jewelServo.setPosition(position);
    }
}