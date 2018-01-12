package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Aayushiron on 1/11/18.
 */

public class AmbitionsBlueNearRelic extends LinearOpMode {
    NewRobotClassFinal robot = new NewRobotClassFinal();
    int trackableViewed;
    double cryptoBoxWidth = 7.63;
    double armOffset = 2.8515;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);
        robot.gyroTurning(135);
        trackableViewed = robot.getTargetColumn();
        robot.gyroTurning(135);
        robot.drive(24);
        robot.drive(((cryptoBoxWidth/2) + (trackableViewed * cryptoBoxWidth)) - armOffset);
        robot.gyroTurning(-90);
        robot.drive(18);
        robot.openClamp();
    }
}
