package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AUTO RED NEAR RELIC")

public class AutonomousRedNearRelic extends Autonomous {
    @Override
    int teamColour() {
        return 1;
    }

    @Override
    boolean nearRelic() {
        return true;
    }
}