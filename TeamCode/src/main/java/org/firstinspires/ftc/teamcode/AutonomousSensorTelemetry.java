package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AUTO SENSOR TELEMETRY")
public class AutonomousSensorTelemetry extends Autonomous {
    @Override
    void autonomous() {
        sensorTelemetryLoop();
    }

    @Override
    int teamColour() {
        return -1;
    }

    @Override
    boolean nearRelic() {
        return true;
    }
}
