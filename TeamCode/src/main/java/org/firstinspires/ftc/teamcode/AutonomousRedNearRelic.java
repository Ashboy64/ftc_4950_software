package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RED NEAR RELIC")

public class AutonomousRedNearRelic extends Autonomous {
    @Override
    RobotHardware.TeamColour teamColour() {
        return RobotHardware.TeamColour.RED;
    }

    @Override
    boolean nearRelic() {
        return true;
    }
}
