package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by rao_a on 1/9/2018.
 */

@com.qualcomm.robotcore.eventloop.opmode.Disabled
public class AmbitionsRedNearRelic extends LinearOpMode {
    NewRobotClassFinal robot =  new NewRobotClassFinal();
    int trackableViewed;
    double cryptoBoxWidth = 7.63;
    double armOffset = 2.8515;

    public void runOpMode() {
        robot.init(hardwareMap, this);
        robot.scoreJewel(0);
        robot.gyroTurning(135);
        trackableViewed = robot.getTargetColumn();
        robot.gyroTurning(135);
        robot.drive(24);
        robot.drive(((cryptoBoxWidth/2) + (trackableViewed * cryptoBoxWidth)) - armOffset);
        robot.gyroTurning(-90);
        robot.openClamp();
        robot.drive(18);
    }
}
