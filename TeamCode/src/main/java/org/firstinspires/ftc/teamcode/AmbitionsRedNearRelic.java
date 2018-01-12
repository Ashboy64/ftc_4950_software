package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by rao_a on 1/9/2018.
 */

public class AmbitionsRedNearRelic extends LinearOpMode {
    NewRobotClassFinal robot =  new NewRobotClassFinal();

    public void runOpMode(){

        robot.init(hardwareMap, this);
        telemetry.addData("ready", "");
        telemetry.update();
        waitForStart();
        scoreJewel();

    }

    private  void jewelTest() {
        robot.setJewelArmPosition(1);

//        try {
//            wait(1000);
//        } catch (InterruptedException e) {
//            telemetry.addData("biggie killed tupac > ", "1");
//            telemetry.update();
//        }

        scoreJewelTest();
        robot.setJewelArmPosition(0);

//        try {
//            wait(1000);
//        } catch (InterruptedException e) {
//            telemetry.addData("biggie killed tupac > ", "2");
//            telemetry.update();
//        }

        telemetry.addData("done!", "");
        telemetry.update();
    }

    private void scoreJewelTest(){
        int detected = robot.getJewelColour();
        robot.gyroEncoders((red() ? (detected == -1 ? -90 : 90) : (detected == 1 ? 90 : -90)));
    }

    private boolean red(){
        return true;
    }

    public void scoreJewel() {
        robot.jewelServo.setPosition(0);
        int detected = robot.getJewelColour();
        robot.gyroEncoders((true ? (detected == -1 ? -90 : 90) : (detected == 1 ? 90 : -90)));
        robot.jewelServo.setPosition(1);
    }
}
