package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by rao_a on 1/18/2018.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "turn test")
@Disabled
public class gyroSignTest extends LinearOpMode {
    NewRobotClassFinal robot =  new NewRobotClassFinal();
    @Override
    public void runOpMode(){
        robot.init(hardwareMap, this);
        telemetry.addData("ready", "");
        telemetry.update();
        while(opModeIsActive()){
            telemetry.addData("Heading: ", robot.gyro.getHeading());
            telemetry.update();
        }
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
        int detected = robot.getJewelColor();
        robot.gyroEncoders((true ? (detected == -1 ? -90 : 90) : (detected == 1 ? 90 : -90)));
    }
}
