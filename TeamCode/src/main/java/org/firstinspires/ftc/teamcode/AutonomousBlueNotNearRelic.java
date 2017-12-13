package org.firstinspires.ftc.teamcode;

public class AutonomousBlueNotNearRelic extends Autonomous {
    @Override
    RobotHardware.TeamColour teamColour() {
        return RobotHardware.TeamColour.BLUE;
    }

    @Override
    boolean nearRelic() {
        return false;
    }
}
