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
        robot.init(hardwareMap, true);

        telemetry.addData(">", "Press play, " + Math.sqrt((12 * 12) + (36 * 36)));
        telemetry.update();

        waitForStart();

        robot.movingForward(Math.sqrt((12 * 12) + (36 * 36)), true);
    }
}
