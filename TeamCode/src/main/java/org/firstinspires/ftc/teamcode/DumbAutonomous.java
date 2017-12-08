package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

/**
 * Created by Aayushiron on 12/5/17.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Dumb ")
public class DumbAutonomous extends LinearOpMode{
    RobotClassFinalUse robot = new RobotClassFinalUse();
    double wheel_diameter = 3.54331; //the diameter of the wheels of our robot.
    double wheel_circumference = Math.PI * wheel_diameter; //the value of π times the wheel diameter
    double distance = Math.sqrt((12 * 12) + (36 * 36));
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);

        telemetry.addData(">", "Press play");
        telemetry.update();

        waitForStart();

        robot.movingForward(distance, opModeIsActive());
    }
}
