package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by rao_a on 1/11/2018.
 */

public class AmbitionsBlueNotNearRelic extends LinearOpMode {
    NewRobotClassFinal robot = new NewRobotClassFinal();
    int trackableViewed;
    double cryptoBoxWidth = 7.63;
    double armOffset = 2.8515;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        robot.scoreJewel(1);
        robot.gyroTurning(135);
        trackableViewed = robot.getTargetColumn();
        robot.gyroEncoders(-135);
        robot.drive(((cryptoBoxWidth / 2) + (trackableViewed * cryptoBoxWidth)) - armOffset);
        robot.gyroEncoders(-90);
        robot.openClamp();
        robot.drive(18.0);
        robot.drive(-100);
    }

}
