package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AUTO BLUE NEAR RELIC")
public class AutonomousBlueNearRelic extends Autonomous {
    @Override
    int teamColour() {
        return -1;
    }

    @Override
    boolean nearRelic() {
        return true;
    }
}
