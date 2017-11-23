package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop", group = "Concept")
public class Teleop extends OpMode {
    public boolean TOUCH_LIMIT_ARM = true;
    public double CLAMP_LIMIT_POWER = 0;

    private RobotInput INPUT;
    private RobotHardware HARDWARE;

    @Override
    public void init() {
        INPUT = new RobotInput(gamepad1, gamepad2);
        HARDWARE = new RobotHardware(hardwareMap);
    }

    @Override
    public void loop() {
        HARDWARE.leftFreeDrive(INPUT.getLeftPower());
        HARDWARE.rightFreeDrive(INPUT.getRightPower());
        HARDWARE.armDrive(INPUT.getArmPower());

        double clampPower = INPUT.getClampPower();
        if (TOUCH_LIMIT_ARM)
        {
            if (HARDWARE.getTouchClosed())
            {
                clampPower = Math.max(clampPower, -CLAMP_LIMIT_POWER);
            }

            else if (HARDWARE.getTouchOpen())
            {
                clampPower = Math.min(clampPower, CLAMP_LIMIT_POWER);
            }
        }
        HARDWARE.setClampPower(INPUT.getClampPower());
    }
}