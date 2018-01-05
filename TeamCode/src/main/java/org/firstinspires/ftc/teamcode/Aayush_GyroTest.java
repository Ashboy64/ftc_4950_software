package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by Aayushiron on 9/12/17.
 */

@Autonomous(name="GyroTest", group="group")
@Disabled
public class Aayush_GyroTest extends LinearOpMode{
    //DcMotor LeftMotor;
    //DcMotor RightMotor;
    GyroSensor gyro;
    public void runOpMode(){

        ElapsedTime opmodeRunTime = new ElapsedTime();


        while (!isStarted()) {
            telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
            telemetry.update();
            idle();
        }

        //LeftMotor = hardwareMap.dcMotor.get("motor1");
        //RightMotor = hardwareMap.dcMotor.get("motor2");

        //RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro = hardwareMap.gyroSensor.get("gyro1");

        gyro.calibrate();

        while (gyro.isCalibrating() && opModeIsActive()) {
            telemetry.addData(">", "Calibrating Sensor. Please Wait.");
            telemetry.update();
        }

        //LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //LeftMotor.setTargetPosition(4000);
        //RightMotor.setTargetPosition(4000);

        //LeftMotor.setPower(1.0);
        //RightMotor.setPower(1.0);

        /* while (LeftMotor.getCurrentPosition()<LeftMotor.getTargetPosition() && RightMotor.getTargetPosition()>RightMotor.getCurrentPosition() && opModeIsActive()) {
           // telemetry.addData(">", gyro.getHeading());
            //telemetry.update();
        }

        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftMotor.setTargetPosition(4000);
        RightMotor.setTargetPosition(4000); */

        //while (gyro.getHeading() < 90) {
          //  LeftMotor.setPower(1.0);
            //RightMotor.setPower(0.0);
            //telemetry.addData(">", gyro.getHeading());
        //}

        /* LeftMotor.setTargetPosition(4000);
        RightMotor.setTargetPosition(4000);

        LeftMotor.setPower(1.0);
        RightMotor.setPower(1.0);

        while (LeftMotor.getCurrentPosition()<LeftMotor.getTargetPosition() && RightMotor.getTargetPosition()>RightMotor.getCurrentPosition() && opModeIsActive()) {
            telemetry.addData(">", gyro.getHeading());
            telemetry.update(); */

        while (opModeIsActive()) {
            telemetry.addData(">", gyro.getHeading());
            telemetry.update();
        }
    }

    public static class Robot
    {
        public DcMotor  leftMotor;
        public DcMotor  rightMotor;
        public DcMotor armMotor;
        ColorSensor colorSensor;
        GyroSensor gyro;
        VuforiaLocalizer vuforia;

        /* Local OpMode members. */
        HardwareMap hwMap  = null;
        private ElapsedTime period  = new ElapsedTime();

        /* Constructor */
        public Robot() {
        }

        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            leftMotor = hardwareMap.dcMotor.get("motor1");
            rightMotor = hardwareMap.dcMotor.get("motor2");
            colorSensor = hardwareMap.colorSensor.get("colorSensor1");
            gyro = hardwareMap.gyroSensor.get("gyro1");

            gyro.calibrate();

            while(gyro.isCalibrating()){
                telemetry.addData(">", "Calibrating gyro. Do not touch");
                telemetry.update();
            }
        }
    }
}
