package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Aayushiron on 1/11/18.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AmbitionsBlueNear")
@com.qualcomm.robotcore.eventloop.opmode.Disabled

public class AmbitionsBlueNearRelic extends LinearOpMode {
    NewRobotClassFinal robot = new NewRobotClassFinal();
    int trackableViewed;
    double cryptoBoxWidth = 7.63;
    double armOffset = 2.8515;
    int vuforiaDegrees = -23;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap, this);
        robot.scoreJewel(1);
        robot.sleep(125);
        robot.gyroTurning(vuforiaDegrees);
        robot.sleep(125);
        trackableViewed = robot.getTargetColumn();
        robot.sleep(125);
        robot.gyroTurning(-(90 + vuforiaDegrees));
        robot.sleep(125);
        robot.drive(24);
        robot.sleep(125);
        robot.drive(((cryptoBoxWidth/2) + (trackableViewed * cryptoBoxWidth)) - armOffset);
        robot.sleep(125);
        robot.gyroTurning(-90);
        robot.sleep(125);
        robot.openClamp();
        robot.sleep(125);
        robot.drive(20);
        robot.sleep(125);
        robot.drive(-5);
    }

}
