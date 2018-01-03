package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Was not Created by Aayushiron on 1/2/18.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Rasputin")

public class gyrotestslalala extends LinearOpMode {

    RobotClassFinalUse robot = new RobotClassFinalUse();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData("k","k");
        telemetry.update();
        waitForStart();

        robot.gyroTurning(90, this);
        telemetry.addData(">", robot.gyro.getHeading());
        telemetry.update();
    }
}
