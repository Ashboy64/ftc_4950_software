package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by mnawani on 9/28/2017.
 */

@Autonomous(name = "colorSensorTest", group = "group")
public class ColorSensorTest extends LinearOpMode {
    ColorSensor color_sensor;

    //@Override
    /*public synchronized void waitForStart() {
        super.waitForStart();
    }*/

    @Override
    public void runOpMode() {

        ElapsedTime opmodeRunTime = new ElapsedTime();


        while (!isStarted()) {
            telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
            telemetry.update();
            idle();
        }

        color_sensor =  hardwareMap.get(ColorSensor.class, "colorSensor1");
        while (opModeIsActive()) {
            color_sensor.enableLed(true);
            telemetry.addData(">", color_sensor.argb());
            telemetry.update();

            //if(color_sensor.argb()!=0){
              //  telemetry.addData("nonzero","");
                //telemetry.update();
            //}
        }
    }
}
