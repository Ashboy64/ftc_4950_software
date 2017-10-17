package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Aayushiron on 10/10/17.
 */

@Autonomous(name="ServoThing", group = "concept")
public class Servo_Aayush extends LinearOpMode{
    Servo servo1;
    @Override
    public void runOpMode(){
        ElapsedTime opmodeRunTime = new ElapsedTime();
       // long startTime = System.currentTimeMillis();

        while (!isStarted()) {
            telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
            telemetry.update();
            idle();
        }
        servo1 = hardwareMap.servo.get("servo1");
        servo1.setPosition(0.0);

        while(System.currentTimeMillis() < 5){
            telemetry.addData("timings", "%0.1f", System.currentTimeMillis());
            telemetry.update();
        }

        while (servo1.getPosition() != 0 && opModeIsActive()) {
            telemetry.addData("GUILLLLLOOTTTIIIINNEE"," ROBESPIERRE");
            telemetry.update();
        }
        telemetry.addData(servo1.getPosition()+ "", "port number");
        telemetry.update();

        servo1.setPosition(1);

        while(System.currentTimeMillis() < 10){
            telemetry.addData("timings", "%0.1f", System.currentTimeMillis());
            telemetry.update();
        }
    }
}
