package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RobotPositionTest", group = "Concept")
public class RobotPositionTest extends OpMode {
    private Robot robot;
    private DoubleGamepad gamepad;

    private Interpolator leftInterpolator = new Interpolator(2);
    private Interpolator rightInterpolator = new Interpolator(2);

    private RobotPosition position = new RobotPosition();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        gamepad = new DoubleGamepad(gamepad1, gamepad2);
        telemetry.setMsTransmissionInterval(100);

        robot.GYRO.startCalibrate();
        robot.IMU.startCalibrate();
        while (robot.GYRO.isCalibrating() || robot.IMU.isCalibrating());

        telemetry.addData("hardware initialization", "complete");
    }

    @Override
    public void loop() {
        updateDrive();
    }

    private void updateDrive() {
        position.update();

        robot.DRIVE_LEFT.setPower(leftInterpolator.value(-gamepad.leftStickY()));
        robot.DRIVE_RIGHT.setPower(rightInterpolator.value(-gamepad.rightStickY()));

        telemetry.addLine(position.toString());
        telemetry.addData("gyro heading", Math.toDegrees(robot.GYRO.getHeading()));
        telemetry.addData("imu heading", Math.toDegrees(robot.IMU.getHeading()));
    }
}