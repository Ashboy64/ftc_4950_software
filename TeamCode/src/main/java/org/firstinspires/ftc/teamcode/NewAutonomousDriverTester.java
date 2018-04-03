package org.firstinspires.ftc.teamcode;

/**
 * Created by justi on 2018-01-03.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "drivertester")
@com.qualcomm.robotcore.eventloop.opmode.Disabled
public class NewAutonomousDriverTester extends NewAutonomous {
    @Override
    public void runOpMode() {
        initRobot();

        //driveTest();
        jewelTest();


        //robot.openClamp();

        //while (opModeIsActive()) {
        //    telemetry.addData("detected colour", robot.getJewelColour());
        //    telemetry.addData("detected target column", robot.getTargetColumn());
        //}
    }

    private  void jewelTest() {
        robot.setJewelArmPosition(1);
        scoreJewelTest();
        robot.setJewelArmPosition(0);
    }

    private void scoreJewelTest(){
        int detected = robot.getJewelColour();
        robot.gyroEncoders((true ? (detected == -1 ? -90 : 90) : (detected == 1 ? 90 : -90)));
    }

    private void turnTest() {
        //testTurn(15);
        //testTurn(-15);
        //testTurn(90);
        //testTurn(-90);
        robot.turn(90);
        telemetry.addData("finished 1","");
        telemetry.update();
      //  robot.turn(-90);
        //testTurn(180);
        //testTurn(-180);
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
        /*throw new IllegalStateException();*/
        return true;
    }

    @Override
    boolean nearRelic() {
        /*throw new IllegalStateException();*/
        return true;
    }
}
