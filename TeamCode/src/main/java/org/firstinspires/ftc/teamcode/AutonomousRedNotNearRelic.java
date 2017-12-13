package org.firstinspires.ftc.teamcode;

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
