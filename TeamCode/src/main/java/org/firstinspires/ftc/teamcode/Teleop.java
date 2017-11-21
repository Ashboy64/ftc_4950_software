package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Teleop", group = "Concept")
public class Teleop extends OpMode {
    private boolean swapTouchSensors = false;
    private boolean swapDriveMotors = false;

    private float drivePower = 0.75f;
    private float drivePowerSlow = 0.25f;
    private float armPower = 1;
    private float clampPower = 1;

    private boolean touchLimit = false;

    private CRServo servoClamp;

    private DcMotor motorArm;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private boolean driveReverse; //reverse driving direction

    private DigitalChannel touchOpen; //arm open
    private boolean touchOpenPressed;
    private DigitalChannel touchClosed; //arm closed
    private boolean touchClosedPressed;

    private boolean bumperLeft;
    private boolean bumperRight;

    private float triggerLeft;
    private float triggerRight;

    private float driveLeft;
    private float driveRight;

    @Override
    public void init() {
        servoClamp = hardwareMap.crservo.get("clampServo");

        touchOpen = hardwareMap.get(DigitalChannel.class, swapTouchSensors ? "tsClosed" : "tsOpen");
        touchClosed = hardwareMap.get(DigitalChannel.class, swapTouchSensors ? "tsOpen" : "tsClosed");

        motorLeft = hardwareMap.dcMotor.get(swapDriveMotors ? "rightMotor" : "leftMotor");
        motorRight = hardwareMap.dcMotor.get(swapDriveMotors ? "leftMotor" : "rightMotor");
        motorArm = hardwareMap.dcMotor.get("armMotor");

        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArm.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void loop() {
        //input processing
        triggerLeft = gamepad1.left_trigger;
        if (triggerLeft == 0) triggerLeft = gamepad2.left_trigger;

        triggerRight = gamepad1.right_trigger;
        if (triggerRight == 0) triggerRight = gamepad2.right_trigger;

        bumperLeft = gamepad1.left_bumper || gamepad2.right_bumper;
        bumperRight = gamepad1.right_bumper || gamepad2.left_bumper;

        driveLeft = gamepad1.left_stick_y;
        if (driveLeft == 0) driveLeft = gamepad2.left_stick_y;
        //if (driveLeft == 0) driveLeft = gamepad1.left_stick_x * drivePowerSlow;
        //if (driveLeft == 0) driveLeft = gamepad2.left_stick_x * drivePowerSlow;

        driveRight = gamepad1.right_stick_y;
        if (driveRight == 0) driveRight = gamepad2.right_stick_y;
        //if (driveRight == 0) driveRight = gamepad1.right_stick_x * -drivePowerSlow;
        //if (driveRight == 0) driveRight = gamepad2.right_stick_x * -drivePowerSlow;

        driveReverse = !gamepad1.y || !gamepad2.y;

        //touch sensor setup
        touchOpen.setMode(DigitalChannel.Mode.INPUT);
        touchClosed.setMode(DigitalChannel.Mode.INPUT);

        touchOpenPressed = touchOpen.getState();
        touchClosedPressed = touchClosed.getState();

        telemetry.addData("Open: " + touchOpenPressed, "Closed: "+ touchClosedPressed);
        telemetry.update();

        //driving
        DcMotor driveLeft = driveReverse ? motorRight : motorLeft;
        DcMotor driveRight = driveReverse ? motorLeft : motorRight;

        int driveDirection = driveReverse ? -1 : 1;

        driveLeft.setPower(driveDirection * drivePower * this.driveLeft);
        driveRight.setPower(driveDirection * drivePower * this.driveRight);

        //arm control
        motorArm.setPower((triggerRight - triggerLeft) * armPower);

        //clamp open/close
        if (bumperRight && (!touchOpenPressed || !touchLimit)) {
            servoClamp.setPower(clampPower);
        } else if (bumperLeft && (!touchClosedPressed || !touchLimit)) {
            servoClamp.setPower(-clampPower);
        } else {
            servoClamp.setPower(0.0);
        }
    }
}