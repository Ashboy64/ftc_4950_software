package org.firstinspires.ftc.teamcode;

/**
 * Created by justi on 2018-01-03.
 * DO NOT EDIT
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BLUE NEAR RELIC")
public class NewAutonomousBlueNearRelic extends NewAutonomous {
    @Override
    boolean red() {
        return false;
    }

    @Override
    boolean nearRelic() {
        return true;
    }
}
