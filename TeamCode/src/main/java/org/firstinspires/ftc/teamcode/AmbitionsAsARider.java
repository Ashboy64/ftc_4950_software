package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by rao_a on 1/9/2018.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AmbitionsAsARider")
//@com.qualcomm.robotcore.eventloop.opmode.Disabled
public class AmbitionsAsARider extends LinearOpMode {
    NewRobotClassFinal robot =  new NewRobotClassFinal();
    @Override
    public void runOpMode(){
        robot.init(hardwareMap, this);
//        while (opModeIsActive()) {
//            robot.imuHeading();
//        }
        //telemetry.update();
       // robot.imuTurning(135);
      //  robot.imuTurning(-145);
        robot.imuTurning(45);
        robot.imuTurning(-45);
        robot.imuTurning(90);
        robot.imuTurning(-90);
        robot.imuTurning(135);
        robot.imuTurning(-135);
        robot.imuTurning(270);
        robot.imuTurning(-270);
        /*robot.imuTurning(350);
        robot.imuTurning(-30);
        robot.imuTurning(55);*/
        /*
        robot.imuTurning(120);
        robot.imuTurning(-25);
        */

    }

    private void jewelTest() {
        robot.setJewelArmPosition(1);

        long time = System.currentTimeMillis();
        while (System.currentTimeMillis() - time < 1000);

        telemetry.addData("done!", "");
        telemetry.update();
        scoreJewelTest();
    }

    private void scoreJewelTest(){
        int detected = robot.getJewelColour();
        robot.gyroEncoders((true ? (detected == -1 ? -90 : 90) : (detected == 1 ? 90 : -90)));
    }

}
