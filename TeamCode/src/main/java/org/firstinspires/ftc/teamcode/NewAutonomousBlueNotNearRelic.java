package org.firstinspires.ftc.teamcode;

/**
 * Created by justi on 2018-01-03.
 * DO NOT EDIT
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BLUE NOT NEAR RELIC")
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
