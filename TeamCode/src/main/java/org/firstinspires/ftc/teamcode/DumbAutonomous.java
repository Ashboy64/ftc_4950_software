package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

/**
 * Created by Aayushiron on 12/5/17.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Dumb ")
@Disabled
public class DumbAutonomous extends LinearOpMode{
    RobotClassFinalUse robot = new RobotClassFinalUse();
    double distance = Math.sqrt((12 * 12) + (36 * 36));
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData(">", "Press play");
        telemetry.update();

        waitForStart();

        robot.movingForward(distance, this);
    }
}
