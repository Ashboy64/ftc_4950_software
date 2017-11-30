package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    private final boolean SWAP_TOUCH_SENSORS = false;
    private final boolean SWAP_DRIVE_MOTORS = false;

    private final boolean ARM_HOLD_POSITION = false;
    private final double ARM_HOLD_THRESHOLD = 0.5;

    private final double ARM_POWER = 1;
    private final double DRIVE_POWER = -1;
    private final double CLAMP_POWER = 1;

    public final int TICKS_PER_MOTOR_REVOLUTION = 2240;

    private final double WHEEL_DIAMETER_MM = 90;
    private final double WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * Math.PI;
    private final double MM_TO_IN = 0.0393700787;
    public final double WHEEL_CIRCUMFERENCE_IN = WHEEL_CIRCUMFERENCE_MM * MM_TO_IN;

    private final CRServo CLAMP_SERVO;

    private final DcMotor ARM_MOTOR;
    private final DcMotor LEFT_MOTOR;
    private final DcMotor RIGHT_MOTOR;

    private final GyroSensor GYRO;

    private final DigitalChannel TOUCH_ARM_OPEN; //arm open
    private final DigitalChannel TOUCH_ARM_CLOSED; //arm closed

    public RobotHardware(HardwareMap hardwareMap) {
        CLAMP_SERVO = hardwareMap.crservo.get("clampServo");

        TOUCH_ARM_OPEN = hardwareMap.get(DigitalChannel.class, SWAP_TOUCH_SENSORS ? "tsClosed" : "tsOpen");
        TOUCH_ARM_CLOSED = hardwareMap.get(DigitalChannel.class, SWAP_TOUCH_SENSORS ? "tsOpen" : "tsClosed");
        TOUCH_ARM_OPEN.setMode(DigitalChannel.Mode.INPUT);
        TOUCH_ARM_CLOSED.setMode(DigitalChannel.Mode.INPUT);

        GYRO = hardwareMap.gyroSensor.get("gyro");

        LEFT_MOTOR = hardwareMap.dcMotor.get(SWAP_DRIVE_MOTORS ? "rightMotor" : "leftMotor");
        LEFT_MOTOR.setDirection(DcMotorSimple.Direction.REVERSE);
        LEFT_MOTOR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RIGHT_MOTOR = hardwareMap.dcMotor.get(SWAP_DRIVE_MOTORS ? "leftMotor" : "rightMotor");
        RIGHT_MOTOR.setDirection(DcMotorSimple.Direction.FORWARD);
        RIGHT_MOTOR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ARM_MOTOR = hardwareMap.dcMotor.get("armMotor");
        ARM_MOTOR.setDirection(DcMotorSimple.Direction.FORWARD);
        ARM_MOTOR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void armDrive(double power) {
        if (ARM_HOLD_POSITION && Math.abs(power) < ARM_HOLD_THRESHOLD) {
            ARM_MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ARM_MOTOR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ARM_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ARM_MOTOR.setTargetPosition(0);
            ARM_MOTOR.setPower(ARM_POWER);
        } else {
            ARM_MOTOR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ARM_MOTOR.setPower(power * ARM_POWER);
        }
    }

    public void leftFreeDrive(double power) {
        LEFT_MOTOR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LEFT_MOTOR.setPower(power * DRIVE_POWER);
    }

    public void rightFreeDrive(double power) {
        RIGHT_MOTOR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RIGHT_MOTOR.setPower(power * DRIVE_POWER);
    }

    public void leftEncoderDrive(int encoderTicks, double power) {
        LEFT_MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFT_MOTOR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFT_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LEFT_MOTOR.setTargetPosition(encoderTicks);
        LEFT_MOTOR.setPower(power * DRIVE_POWER);
    }

    public void rightEncoderDrive(int encoderTicks, double power) {
        RIGHT_MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHT_MOTOR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RIGHT_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RIGHT_MOTOR.setTargetPosition(encoderTicks);
        RIGHT_MOTOR.setPower(power * DRIVE_POWER);
    }

    public void setClampPower(double power) {
        CLAMP_SERVO.setPower(power * CLAMP_POWER);
    }

    public boolean getTouchOpen() {
        return TOUCH_ARM_OPEN.getState();
    }

    public boolean getTouchClosed() {
        return TOUCH_ARM_CLOSED.getState();
    }

    public int gyroHeading() {
        return GYRO.getHeading();
    }
}