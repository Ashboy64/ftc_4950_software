package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AUTO TESTING")
public class AutonomousTesting extends Autonomous {
    @Override
    void autonomous() {
        useGyro = false;

        ElapsedTime drive = new ElapsedTime();
        freeDrive(DRIVE_POWER, DRIVE_POWER);
        while (drive.milliseconds() < 2000) {
            sensorTelemetry("driving " + drive.milliseconds());
        }
        stopDrive();
        turn(90, TURN_POWER);

        drive = new ElapsedTime();
        freeDrive(DRIVE_POWER, DRIVE_POWER);
        while (drive.milliseconds() < 2000) {
            sensorTelemetry("driving " + drive.milliseconds());
        }
        stopDrive();
        turn(90, TURN_POWER);

        useGyro = true;

        drive = new ElapsedTime();
        freeDrive(DRIVE_POWER, DRIVE_POWER);
        while (drive.milliseconds() < 2000) {
            sensorTelemetry("driving " + drive.milliseconds());
        }
        stopDrive();
        turn(90, TURN_POWER);

        drive = new ElapsedTime();
        freeDrive(DRIVE_POWER, DRIVE_POWER);
        while (drive.milliseconds() < 2000) {
            sensorTelemetry("driving " + drive.milliseconds());
        }
        stopDrive();
        turn(90, TURN_POWER);
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
