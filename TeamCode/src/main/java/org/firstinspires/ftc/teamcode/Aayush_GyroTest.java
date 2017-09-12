package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Aayushiron on 9/12/17.
 */

@Autonomous(name="First autonomous", group="group")
@Disabled
public class Aayush_GyroTest extends LinearOpMode{
    DcMotor LeftMotor;
    DcMotor RightMotor;
    GyroSensor gyro;
    public void runOpMode(){
        LeftMotor = hardwareMap.dcMotor.get("motor1");
        RightMotor = hardwareMap.dcMotor.get("motor2");
        gyro = hardwareMap.gyroSensor.get("gyro1");

        gyro.calibrate();

        while (gyro.isCalibrating() && opModeIsActive()) {

        }

        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftMotor.setTargetPosition(4000);
        RightMotor.setTargetPosition(4000);

        LeftMotor.setPower(1.0);
        RightMotor.setPower(1.0);

        while (LeftMotor.getCurrentPosition()<LeftMotor.getTargetPosition() && RightMotor.getTargetPosition()>RightMotor.getCurrentPosition() && opModeIsActive()) {

        }

        LeftMotor.setTargetPosition(4000);
        RightMotor.setTargetPosition(4000);

        while (gyro.getHeading() < 90) {
            LeftMotor.setPower(1.0);
            RightMotor.setPower(0.0);
        }

        LeftMotor.setTargetPosition(4000);
        RightMotor.setTargetPosition(4000);

        LeftMotor.setPower(1.0);
        RightMotor.setPower(1.0);

        while (LeftMotor.getCurrentPosition()<LeftMotor.getTargetPosition() && RightMotor.getTargetPosition()>RightMotor.getCurrentPosition() && opModeIsActive()) {

        }
        // telemetry.addData("Gyro heading:", gyroHeading);
    }
}
