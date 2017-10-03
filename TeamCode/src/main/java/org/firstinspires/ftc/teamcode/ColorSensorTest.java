package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by mnawani on 9/28/2017.
 */

@Autonomous(name = "colorSensorTest", group = "group")
public class ColorSensorTest extends LinearOpMode {
    ColorSensor color_sensor;
    @Override
    public void runOpMode() {
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
