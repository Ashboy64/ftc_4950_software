package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AUTO BLUE NOT NEAR RELIC")
public class AutonomousBlueNotNearRelic extends Autonomous {
    @Override
    int teamColour() {
        return -1;
    }

    @Override
    boolean nearRelic() {
        return false;
    }
}
