package org.firstinspires.ftc.teamcode;

/**
 * Created by justi on 2018-01-03.
 */

public class NewAutonomousDriverTester extends NewAutonomous {
    @Override
    public void runOpMode() {
        initRobot();

        driveTest();

        turnTest();

        robot.openClamp();

        while (opModeIsActive()) {
            telemetry.addData("detected colour", robot.getJewelColour());
            telemetry.addData("detected target column", robot.getTargetColumn());
        }
    }

    private void turnTest() {
        testTurn(15);
        testTurn(-15);
        testTurn(90);
        testTurn(-90);
        testTurn(180);
        testTurn(-180);
    }

    private void driveTest() {
        testDrive(4);
        testDrive(-4);
        testDrive(0);
    }

    private void testDrive(double inches) {
        telemetry.addLine("driving " + inches + " inches");
        robot.drive(inches);
    }

    private void testTurn(int degrees) {
        telemetry.addLine("turning " + degrees + " degrees");
        robot.turn(degrees);
    }

    @Override
    boolean red() {
        throw new IllegalStateException();
    }

    @Override
    boolean nearRelic() {
        throw new IllegalStateException();
    }
}
