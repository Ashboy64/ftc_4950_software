package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by rao_a on 1/9/2018.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AmbitionsAsARider")
public class AmbitionsAsARider extends LinearOpMode {
    NewRobotClassFinal robot =  new NewRobotClassFinal();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        jewelTest();
    }

    private  void jewelTest() {
        robot.setJewelArmPosition(1);

        try {
            wait(1000);
        } catch (InterruptedException e) {
            telemetry.addData("biggie killed tupac > ", "1");
            telemetry.update();
        }

        scoreJewelTest();
        robot.setJewelArmPosition(0);

        try {
            wait(1000);
        } catch (InterruptedException e) {
            telemetry.addData("biggie killed tupac > ", "2");
            telemetry.update();
        }

        telemetry.addData("done!", "");
        telemetry.update();
    }

    private void scoreJewelTest(){
        int detected = robot.getJewelColour();
        robot.gyroEncoders((true ? (detected == -1 ? -90 : 90) : (detected == 1 ? 90 : -90)));
    }

}
