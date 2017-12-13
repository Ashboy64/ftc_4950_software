package org.firstinspires.ftc.teamcode;

public class AutonomousBlueNearRelic extends Autonomous {
    @Override
    RobotHardware.TeamColour teamColour() {
        return RobotHardware.TeamColour.BLUE;
    }

    @Override
    boolean nearRelic() {
        return true;
    }
}
