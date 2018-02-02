package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by rao_a on 1/9/2018.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AmbitionsAsARider")
//@com.qualcomm.robotcore.eventloop.opmode.Disabled
public class AmbitionsAsARider extends LinearOpMode {
    NewRobotClassFinal robot =  new NewRobotClassFinal();
    @Override
    public void runOpMode(){
        robot.init(hardwareMap, this);
//        while (opModeIsActive()) {
//            robot.imuHeading();
//        }
        //telemetry.update();
       // robot.imuTurning(135);
      //  robot.imuTurning(-145);

        robot.gyroTurning(45);
        robot.gyroTurning(-45);
        robot.gyroTurning(90);
        robot.gyroTurning(-90);
        robot.gyroTurning(135);
        robot.gyroTurning(-135);
        robot.gyroTurning(270);
        robot.gyroTurning(-270);
        /*robot.imuTurning(350);
        robot.imuTurning(-30);
        robot.imuTurning(55);*/
        /*
        robot.imuTurning(120);
        robot.imuTurning(-25);
        */
        telemetry.addData(">", "Scoring jewel");
        telemetry.update();
        robot.sleep(1000);
        //robot.scoreJewel(1);
    }
}
