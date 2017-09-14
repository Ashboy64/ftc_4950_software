package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

@Autonomous(name="FirstAutonomousCantSpellGyro", group="group")
//@Disabled
public class gryo extends LinearOpMode{
    GyroSensor gyro;
    public void runOpMode () {
        gyro = hardwareMap.gyroSensor.get("gryo");
        gyro.calibrate();
        while(gyro.isCalibrating()){

        }
        telemetry.addData("heading", gyro.getHeading());
        telemetry.update();
    }

}
