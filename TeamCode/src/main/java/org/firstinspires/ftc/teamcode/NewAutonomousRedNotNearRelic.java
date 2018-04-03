package org.firstinspires.ftc.teamcode;

/**
 * Created by justi on 2018-01-03.
 * DO NOT EDIT
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RED NOT NEAR RELIC")
@com.qualcomm.robotcore.eventloop.opmode.Disabled
public class NewAutonomousRedNotNearRelic extends NewAutonomous {
    @Override
    boolean red() {
        return true;
    }

    @Override
    boolean nearRelic() {
        return false;
    }
}
