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

        while (!isStarted()) {
            telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
            telemetry.update();
            idle();
        }
        servo1 = hardwareMap.servo.get("servo1");

        double counterStart = opmodeRunTime.seconds();

        servo1.setPosition(0.0);


         while(opmodeRunTime.seconds() < counterStart + 5){
            telemetry.addData("timings", "In first loop");
            telemetry.update();
        }

        counterStart = opmodeRunTime.seconds();

        servo1.setPosition(1);

        while(opmodeRunTime.seconds() < counterStart + 5){
            telemetry.addData("timings", "Rachit is the best");
            telemetry.update();
        }
    }
}
