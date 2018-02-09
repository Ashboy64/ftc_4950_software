package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by rao_a on 1/11/2018.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AmbitionsBlueNotNear")
public class AmbitionsBlueNotNearRelic extends LinearOpMode {
    NewRobotClassFinal robot = new NewRobotClassFinal();
    int trackableViewed;
    double cryptoBoxWidth = 7.63;
    double armOffset = 2.8515;
    int vuforiaDegrees = -23;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        robot.sleep(125);
        robot.scoreJewel(1);
        robot.sleep(125);
        robot.gyroTurning(vuforiaDegrees);
        robot.sleep(125);
        trackableViewed = robot.getTargetColumn();
        robot.sleep(125);
        robot.gyroTurning(-1*(90 + robot.gyro.getHeading()));
        robot.sleep(125);
        robot.drive(24);
        robot.sleep(125);
        robot.gyroTurning(90);
        robot.sleep(125);
        robot.drive(((cryptoBoxWidth / 2) + (trackableViewed * cryptoBoxWidth)) - armOffset);
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
