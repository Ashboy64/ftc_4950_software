package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BLUE NEAR RELIC")
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
