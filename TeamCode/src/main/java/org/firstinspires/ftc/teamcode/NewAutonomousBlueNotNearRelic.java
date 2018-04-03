package org.firstinspires.ftc.teamcode;

/**
 * Created by justi on 2018-01-03.
 * DO NOT EDIT
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BLUE NOT NEAR RELIC")
@com.qualcomm.robotcore.eventloop.opmode.Disabled
public class NewAutonomousBlueNotNearRelic extends NewAutonomous {
    @Override
    boolean red() {
        return false;
    }

    @Override
    boolean nearRelic() {
        return false;
    }
}
