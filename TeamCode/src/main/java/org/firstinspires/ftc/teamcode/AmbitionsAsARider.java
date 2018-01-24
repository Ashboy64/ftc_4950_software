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
        telemetry.addData("ready", "");
        telemetry.update();
       // robot.gyroTurning(135);
      //  robot.gyroTurning(-145);
        robot.gyroTurning(45);
        robot.gyroTurning(-45);
        robot.gyroTurning(90);
        robot.gyroTurning(-90);
        robot.gyroTurning(135);
        robot.gyroTurning(-135);
        robot.gyroTurning(270);
        robot.gyroTurning(-270);
        /*robot.gyroTurning(350);
        robot.gyroTurning(-30);
        robot.gyroTurning(55);*/
        /*
        robot.gyroTurning(120);
        robot.gyroTurning(-25);
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
