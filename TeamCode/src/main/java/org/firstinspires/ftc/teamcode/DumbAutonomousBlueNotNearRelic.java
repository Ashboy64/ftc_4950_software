package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Aayushiron on 12/5/17.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Dumb blue not near relic")
@Disabled
public class DumbAutonomousBlueNotNearRelic extends LinearOpMode {
    RobotClassFinalUse robot = new RobotClassFinalUse();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, opModeIsActive());

        robot.movingForward(24, opModeIsActive());
        robot.movingForward(12, opModeIsActive());
        robot.gyroTurning(-90, opModeIsActive());
        robot.movingForward(6, opModeIsActive());
    }
}
