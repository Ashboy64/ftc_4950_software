package org.firstinspires.ftc.teamcode;

/**
 * Created by justi on 2018-01-03.
 * DO NOT EDIT
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RED NEAR RELIC")
@com.qualcomm.robotcore.eventloop.opmode.Disabled
public class NewAutonomousRedNearRelic extends NewAutonomous {
    @Override
    boolean red() {
        return true;
    }

    @Override
    boolean nearRelic() {
        return true;
    }
}
