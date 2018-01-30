package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AUTO TESTING")
public class AutonomousTesting extends Autonomous {
    @Override
    void autonomous() {
        turn(45, TURN_POWER);
        sleep();

        turn(-45, TURN_POWER);
        sleep();

        turn(90, TURN_POWER);
        sleep();

        turn(-90, TURN_POWER);
        sleep();

        turn(135, TURN_POWER);
        sleep();

        turn(-135, TURN_POWER);
        sleep();
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
