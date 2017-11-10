package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by yinghuang on 10/12/17.
 */
@Autonomous(name = "ServoCarter", group = "concept")
public class Servo_Carter extends LinearOpMode{
    Servo servo1;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime opmodeRunTime = new ElapsedTime();
        while (!isStarted()) {
            telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
            telemetry.update();
            idle();
        }

        servo1 = hardwareMap.servo.get("servo1");
        servo1.setPosition(0);
        while(opModeIsActive() && servo1.getPosition() != 0){

        }

        servo1.setPosition(1);
        while(opModeIsActive() && servo1.getPosition() != 1){

        }
    }
}
