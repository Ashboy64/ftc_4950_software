package org.firstinspires.ftc.teamcode;

import android.graphics.LinearGradient;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by rao_a on 12/14/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RaChItBaLlEr")
@com.qualcomm.robotcore.eventloop.opmode.Disabled

public class GyroTestsCoolKid extends LinearOpMode {

    RobotClassFinalUse robot = new RobotClassFinalUse();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData("ready", "runrich");
        telemetry.update();
        waitForStart();
        robot.gyroTurning(90, this);
    }
}
