package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AUTO RED NOT NEAR RELIC")
public class AutonomousRedNotNearRelic extends Autonomous {
    @Override
    int teamColour() {
        return 1;
    }

    @Override
    boolean nearRelic() {
        return false;
    }
}
