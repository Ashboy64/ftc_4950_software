package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class Autonomous extends LinearOpMode {
    private RobotInput INPUT;
    private RobotHardware HARDWARE;
    private RobotAutonomousDriver DRIVER;

    @Override
    public void runOpMode() throws InterruptedException {
        INPUT = new RobotInput(gamepad1, gamepad2);
        HARDWARE = new RobotHardware(hardwareMap);
        DRIVER = new RobotAutonomousDriver(HARDWARE);

        //autonomous code here
    }

    abstract boolean blue();

    abstract boolean nearRelic();
}
