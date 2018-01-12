package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by rao_a on 1/9/2018.
 */

public class AmbitionsRedNearRelic extends LinearOpMode {
    NewRobotClassFinal robot =  new NewRobotClassFinal();
    int trackableViewed;
    double cryptoBoxWidth = 7.63;
    double armOffset = 2.8515;

    public void runOpMode(){
        robot.init(hardwareMap, this);
        robot.gyroTurning(135);
        trackableViewed = robot.getTargetColumn();
        robot.gyroTurning(135);
        robot.drive(24);
        robot.drive(((cryptoBoxWidth/2) + (trackableViewed * cryptoBoxWidth)) - armOffset);
        robot.gyroTurning(-90);
        robot.openClamp();
        robot.drive(18);
    }

    public void scoreJewel(){

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
}