package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by rao_a on 1/18/2018.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "turn test")
public class gyroSignTest extends LinearOpMode {
    NewRobotClassFinal robot =  new NewRobotClassFinal();
    @Override
    public void runOpMode(){
        robot.init(hardwareMap, this);
        telemetry.addData("ready", "");
        telemetry.update();
        while(opModeIsActive()){
            telemetry.addData("Heading: ", robot.gyro.getHeading());
            telemetry.update();
        }
    }
}
