package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.sun.tools.javac.util.Constants.format;

public class RobotHardware {
    public enum TargetColumn {UNDEFINED, LEFT, CENTRE, RIGHT}

    public enum TeamColour {UNDEFINED, BLUE, RED}

    private static final int JEWEL_COLOUR_THRESHOLD = 64;

    public static final double TICKS_PER_MOTOR_REVOLUTION = 1120; //2240

    public static final double ARM_SPROCKET_RATIO = 15.0 / 54;
    public static final double WHEEL_CIRCUMFERENCE_IN = 90 * 0.0393701 * Math.PI;
    public static final double DRIVE_GEAR_RATIO = 45.0 / 30;
    public static final double TURN_DISTANCE = 12.75 * Math.PI;

    public static final double GLYPH_OFFSET_RIGHT = 2.8515;
    public static final double GLYPH_OFFSET_FORWARD = 7.588;

    private final CRServo CLAMP_SERVO;

    private final DcMotor ARM_MOTOR;
    private final DcMotor LEFT_MOTOR;
    private final DcMotor RIGHT_MOTOR;

    private final GyroSensor GYRO;

    private final DigitalChannel TOUCH_ARM_OPEN;
    private final DigitalChannel TOUCH_ARM_CLOSED;

    private final Servo JEWEL_SERVO;
    private final ColorSensor COLOUR_JEWEL;

    private final HardwareMap HARDWARE_MAP;
    private VuforiaTrackables vuforiaRelicTrackables;

    private final LinearOpMode LINEAR_OP_MODE;

    public RobotHardware(HardwareMap hardwareMap, LinearOpMode linearOpMode) {
        HARDWARE_MAP = hardwareMap;
        LINEAR_OP_MODE = linearOpMode;

        CLAMP_SERVO = hardwareMap.crservo.get("clampServo");

        TOUCH_ARM_OPEN = hardwareMap.get(DigitalChannel.class, "tsOpen");
        TOUCH_ARM_CLOSED = hardwareMap.get(DigitalChannel.class, "tsClosed");
        TOUCH_ARM_OPEN.setMode(DigitalChannel.Mode.INPUT);
        TOUCH_ARM_CLOSED.setMode(DigitalChannel.Mode.INPUT);

        GYRO = hardwareMap.gyroSensor.get("gyro");

        LEFT_MOTOR = hardwareMap.dcMotor.get("leftMotor");
        RIGHT_MOTOR = hardwareMap.dcMotor.get("rightMotor");
        LEFT_MOTOR.setDirection(DcMotorSimple.Direction.FORWARD);
        RIGHT_MOTOR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorZeroPowerBrake(false);

        ARM_MOTOR = hardwareMap.dcMotor.get("armMotor");
        ARM_MOTOR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetArm();

        JEWEL_SERVO = hardwareMap.servo.get("jewelServo");
        COLOUR_JEWEL = hardwareMap.colorSensor.get("teamColour");
    }

    public RobotHardware(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public void motorZeroPowerBrake(boolean brake) {
        for (DcMotor motor : new DcMotor[]{LEFT_MOTOR, RIGHT_MOTOR}) {
            if (brake) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }
    }

    private void resetArm() {
        ARM_MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ARM_MOTOR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ARM_MOTOR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void armDrive(double power) {
        ARM_MOTOR.setPower(power);
    }

    public void freeDrive(double left, double right) {
        driveWithEncoders(false);
        LEFT_MOTOR.setPower(left);
        RIGHT_MOTOR.setPower(right);
    }

    public void stopDrive() {
        freeDrive(0, 0);
    }

    public void gyroTurn(int angle, double power) {
        int current = angleNormalize(gyroHeading());
        int target = angleNormalize(current + angle);
        int delta = angleNormalize(target - current);

        int direction = delta > 0 ? 1 : -1;

        while (active() && angleNormalize(gyroHeading()) != target) {
            freeDrive(direction * power, -direction * power);
        }
        stopDrive();
    }

    public void encoderTurn(double angle, double power) {
        double robotRotations = angle / 360;
        double distance = robotRotations * TURN_DISTANCE;

        encoderDrive(distance, -distance, power);
    }

    public void encoderDrive(double distance, double power) {
        encoderDrive(distance, distance, power);
    }

    private void encoderDrive(double leftInches, double rightInches, double power) {
        double leftRotations = leftInches / WHEEL_CIRCUMFERENCE_IN;
        double rightRotations = rightInches / WHEEL_CIRCUMFERENCE_IN;

        double leftShaft = leftRotations / DRIVE_GEAR_RATIO;
        double rightShaft = rightRotations / DRIVE_GEAR_RATIO;

        int leftTicks = (int) Math.round(leftShaft * TICKS_PER_MOTOR_REVOLUTION);
        int rightTicks = (int) Math.round(rightShaft * TICKS_PER_MOTOR_REVOLUTION);

        encoderDriveTicks(leftTicks, rightTicks, power);
    }

    private void encoderDriveTicks(int left, int right, double power) {
        driveWithEncoders(true);
        LEFT_MOTOR.setTargetPosition(left);
        LEFT_MOTOR.setPower(power);
        RIGHT_MOTOR.setTargetPosition(right);
        RIGHT_MOTOR.setPower(power);
        while (active() && (LEFT_MOTOR.isBusy() || RIGHT_MOTOR.isBusy())) ;
    }

    private void driveWithEncoders(boolean enabled) {
        for (DcMotor motor : new DcMotor[]{LEFT_MOTOR, RIGHT_MOTOR}) {
            if (enabled) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    public void setClampPower(double power) {
        CLAMP_SERVO.setPower(power);
    }

    public boolean getTouchOpen() {
        return !TOUCH_ARM_OPEN.getState();
    }

    public boolean getTouchClosed() {
        return !TOUCH_ARM_CLOSED.getState();
    }

    public double armPosition() { //should return 0 when arm is down, 180 when arm is up
        return ARM_MOTOR.getCurrentPosition()
                / TICKS_PER_MOTOR_REVOLUTION
                * ARM_SPROCKET_RATIO
                * 360;
    }

    public void openClamp() {
        CLAMP_SERVO.setPower(-1);
        while (active() && !getTouchOpen()) ;
        CLAMP_SERVO.setPower(0);
    }

    public void gyroCalibrate() {
        GYRO.calibrate();
        while (active() && GYRO.isCalibrating()) ;
    }

    public int gyroHeading() {
        return GYRO.getHeading();
    }

    private int angleNormalize(int angle) {
        angle %= 360;
        if (angle < -180) {
            return angle + 360;
        } else if (angle > 180) {
            return angle - 360;
        } else {
            return angle;
        }
    }

    public TargetColumn targetColumn() {
        for (int i = 0; i < vuforiaRelicTrackables.size(); i++) {
            VuforiaTrackable vuforiaRelicTemplate = vuforiaRelicTrackables.get(i);

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(vuforiaRelicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                return fromVuMark(vuMark);
            }
        }

        return TargetColumn.UNDEFINED;
    }

    private TargetColumn fromVuMark(RelicRecoveryVuMark vuMark) {
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            return TargetColumn.LEFT;
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            return TargetColumn.CENTRE;
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            return TargetColumn.RIGHT;
        } else {
            return TargetColumn.UNDEFINED;
        }
    }

    public void initVuforia() {
        int cameraMonitorViewId = HARDWARE_MAP.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", HARDWARE_MAP.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AWnZ5xz/////AAAAGYmbM16TXEdKscTtfaECY6FzIRnxfc6SV0uwUV+dwPVIWbGyu9567BTp2qzh6ohnawdFrbL290ECRr04ew/QX0Q90SUrGh52+s55yVFPN429A93YJm6AlnV/TEJKb8omxdlqC+Hfy0SLPZSu+UEq9xQMOIfeW+OiRNQyFlUTZNCtQDNuK5jwObgulF83zrexs+c95Cd1jU7PnoX+NgHPjmUWS5H+WVr4yZsewES+oa0jRjGrcGU0/P5USRnqVbKh4976SNjPBGy6fanxJZmQb2Pam56UROtERcdaPDSWg4Nrr0MFlHCvi3PcfyLfdPtBW06JZGWBXu23VJCBQFw3SxGm/IO057P4kbTFti3W5xkU";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        vuforiaRelicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        vuforiaRelicTrackables.activate();
    }

    public TeamColour jewelColour() {
        int red = COLOUR_JEWEL.red();
        int blue = COLOUR_JEWEL.blue();

        if (Math.abs(red - blue) < JEWEL_COLOUR_THRESHOLD) {
            return TeamColour.UNDEFINED;
        } else if (red > blue) {
            return TeamColour.RED;
        } else {
            return TeamColour.BLUE;
        }
    }

    public void setJewelArmPosition(double position) {
        JEWEL_SERVO.setPosition(position);
    }

    private boolean active() {
        return LINEAR_OP_MODE == null || LINEAR_OP_MODE.opModeIsActive();
    }
}