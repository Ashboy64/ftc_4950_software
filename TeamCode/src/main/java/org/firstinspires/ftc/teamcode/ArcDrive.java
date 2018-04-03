package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "ArcDrive")
public class ArcDrive extends LinearOpMode {
    private Robot robot;
    private RobotPosition position;

    private final double DRIVE_POWER = 0.125;

    private double leftRadius;
    private double rightRadius;
    private double leftDistance;
    private double rightDistance;

    private DoubleGamepad input;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        position = new RobotPosition();

        input = new DoubleGamepad(gamepad1, gamepad2);

        telemetry.setMsTransmissionInterval(100);

        waitForStart();

        driveStraight(24);
        //driveArc(2 * Math.PI, 18);

        sleep(1000);

        telemetry.addLine("calibrating");
        telemetry.update();

        //driveArc(Math.PI, -24);

        robot.IMU.startCalibrate();
        robot.GYRO.startCalibrate();

        while (robot.GYRO.isCalibrating() && opModeIsActive()) ;

        telemetry.addLine("calibrated");
        telemetry.update();

        while (opModeIsActive()) {
            if (input.a()) {
                telemetry.addData("gyro", robot.GYRO.getHeading());
                telemetry.addData("imu", robot.IMU.getHeading());
                telemetry.update();
            }
        }
    }

    private void driveArc(double angle, double radius) {
        radius *= 6.0 / 7 * 48 / 45;

        angle *= (2 * Math.PI + 0.6) / (2 * Math.PI);
        angle = Math.abs(angle);

        if (angle == 0) {
            return;
        }

        leftRadius = radius + Robot.DRIVE_WIDTH / 2;
        rightRadius = radius - Robot.DRIVE_WIDTH / 2;

        leftDistance = Math.abs(leftRadius * angle);
        rightDistance = Math.abs(rightRadius * angle);

        drive(leftDistance, rightDistance);
    }

    private void driveStraight(double distance) {
        drive(distance, distance);
    }

    private void drive(double left, double right) {
        robot.DRIVE_LEFT.setTargetPosition(robot.DRIVE_LEFT.getCurrentPosition() + Robot.inchesToEncoderTicks(left));
        robot.DRIVE_RIGHT.setTargetPosition(robot.DRIVE_RIGHT.getCurrentPosition() + Robot.inchesToEncoderTicks(right));

        robot.DRIVE_LEFT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.DRIVE_RIGHT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double leftPower = DRIVE_POWER * left / Math.max(left, right);
        double rightPower = DRIVE_POWER * right / Math.max(left, right);

        robot.DRIVE_LEFT.setPower(leftPower);
        robot.DRIVE_RIGHT.setPower(rightPower);

        while (robot.DRIVE_LEFT.isBusy() && robot.DRIVE_RIGHT.isBusy() && opModeIsActive()) {
            telemetry();
        }
    }

    private void telemetry() {
        position.update();
        telemetry.addData("position", position.toString());
        telemetry.addData("left motor tick", robot.DRIVE_LEFT.getCurrentPosition());
        telemetry.addData("right motor tick", robot.DRIVE_RIGHT.getCurrentPosition());
        telemetry.addData("radii", "left %.2f right %.2f", leftRadius, rightRadius);
        telemetry.addData("distances", "left %.2f right %.2f", leftDistance, rightDistance);
        telemetry.update();
    }
}
