package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AUTO TEST")
public class AutonomousTest extends Autonomous {
    public void runOpMode() throws InterruptedException {
        HARDWARE = new RobotHardware(hardwareMap, this);
        INPUT = new RobotInput(gamepad1, gamepad2);

        HARDWARE.motorZeroPowerBrake(true);
        HARDWARE.gyroCalibrate();

        testing();
    }

    private void testing() {
        while (opModeIsActive()) {
            HARDWARE.encoderDrive(4, DRIVE_POWER);
            HARDWARE.encoderDrive(-4, DRIVE_POWER);
            HARDWARE.turn(15, TURN_POWER);
            HARDWARE.turn(-15, TURN_POWER);
        }
    }

    @Override
    RobotHardware.TeamColour teamColour() {
        return RobotHardware.TeamColour.BLUE;
    }

    @Override
    boolean nearRelic() {
        return true;
    }
}
