package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ParametricDrive")
public class ParametricDrive extends LinearOpMode {
    private Robot robot;
    private RobotPosition position = new RobotPosition();

    private Interpolator leftInterpolator = new Interpolator(1);
    private Interpolator rightInterpolator = new Interpolator(1);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        telemetry.setMsTransmissionInterval(100);

        waitForStart();

        pathFollow();
    }

    private void pathFollow() {
        int lookahead = 4;
        double angleCorrectionCoefficient = 2 / Math.PI;

        ParametricPath path = new Line(new Point(0, 0), new Point(36, 36));

        while (opModeIsActive()) {
            position.update();

            telemetry.addData("position", position.toString());

            double t = path.nearestT(position.getPosition());
            telemetry.addData("t", t);

            Point target = path.getPoint(t + lookahead);
            telemetry.addData("target", target);

            Vector translation = position.getPosition().vectorTo(target);
            telemetry.addData("vector", translation);

            double angleCorrection = Utils.angleDifference(translation.getAngle(), position.getAngle()) * angleCorrectionCoefficient;
            telemetry.addData("angle correction", angleCorrection);

            double leftPower = leftInterpolator.value(1 + angleCorrection);
            double rightPower = rightInterpolator.value(1 - angleCorrection);
            robot.DRIVE_LEFT.setPower(leftPower);
            robot.DRIVE_LEFT.setPower(rightPower);
            telemetry.addData("motor powers", new Vector(leftPower, rightPower));

            telemetry.update();
        }
    }
}
