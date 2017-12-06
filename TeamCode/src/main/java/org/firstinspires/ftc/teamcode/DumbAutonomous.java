package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

/**
 * Created by Aayushiron on 12/5/17.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Dumb ")
public class DumbAutonomous extends LinearOpMode{
    RobotClassFinalUse robot = new RobotClassFinalUse();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, opModeIsActive());

        waitForStart();

        telemetry.addData(">", "Press play");
        telemetry.update();

        robot.movingForward(Math.sqrt((12 * 12) + (36 * 36)), true);
    }
}
