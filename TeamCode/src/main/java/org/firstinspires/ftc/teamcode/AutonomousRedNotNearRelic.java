package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AUTO RED NOT NEAR RELIC")
public class AutonomousRedNotNearRelic extends Autonomous {
    @Override
    RobotHardware.TeamColour teamColour() {
        return RobotHardware.TeamColour.RED;
    }

    @Override
    boolean nearRelic() {
        return false;
    }
}
