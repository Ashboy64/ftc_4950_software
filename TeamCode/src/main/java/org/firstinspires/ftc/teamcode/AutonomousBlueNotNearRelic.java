package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BLUE NOT NEAR RELIC")
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
