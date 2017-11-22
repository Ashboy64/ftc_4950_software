package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by Aayushiron on 11/21/17.
 */

public class TestAutonomous extends LinearOpMode {

    double wheel_diameter = 3.5; //the diameter of the wheels of our robot.
    double wheel_circumference = Math.PI * wheel_diameter; //the value of π times the wheel diameter
    DcMotor leftMotor; //allows for control of the robot’s left motor’s actions
    DcMotor rightMotor; //allows for control of the robot’s right motor’s actions
    int ticksPerRevolution = 2240; //the amount of ticks the encoder takes to revolve one wheel
    DcMotor armMotor; //allows for control of our robot’s arm
    GyroSensor gyro; //receives information about the direction of our robot
    VuforiaLocalizer vuforia; //an image-processing library that allows us to analyze pictures
    CRServo clampServo;
    double armWaiting = 2.0;
    float getToJewel = 0;
    //CRServo jewelServo;
    //ColorSensor colorSensor;
    int trackableViewed;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        gyro = hardwareMap.gyroSensor.get("gyro");
        clampServo = hardwareMap.crservo.get("clampServo");
        //colorSensor = hardwareMap.colorSensor.get("colorSensor");
        //jewelServo = hardwareMap.crservo.get("jewelServo");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        movingForward(5);
        gyroTurning(90);
        wait(1000);
        gyroTurning(-90);
        movingForward(-5);
    }

    public void movingForward(double distance) {
        int encoderTicks = (int) Math.ceil((distance/wheel_circumference) * ticksPerRevolution);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition(encoderTicks);
        rightMotor.setTargetPosition(encoderTicks);

        if (distance < 0) {
            leftMotor.setPower(-1);
            rightMotor.setPower(-1);
        } else if (distance >= 0) {
            leftMotor.setPower(1);
            rightMotor.setPower(1);
        }

        while (leftMotor.getCurrentPosition() < leftMotor.getTargetPosition() && rightMotor.getCurrentPosition() < rightMotor.getTargetPosition()) {

        }

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void armMoving() {
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(1120);
        armMotor.setPower(1);
        while (armMotor.getCurrentPosition() < armMotor.getTargetPosition() && opModeIsActive()) {

        }
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armGrabbing();
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(1120);
        armMotor.setPower(1);
    }

    public void armGrabbing() {
        ElapsedTime opmodeRunTime = new ElapsedTime();
        clampServo.setPower(1);
        while (opmodeRunTime.seconds() < armWaiting) {
            telemetry.addData("waiting for arm to get to position", "");
            telemetry.update();
            idle();
        }
        clampServo.setPower(0);
    }

    public void armRelease() {
        ElapsedTime opmodeRunTime = new ElapsedTime();
        clampServo.setDirection(DcMotorSimple.Direction.REVERSE);
        clampServo.setPower(1);
        while (opmodeRunTime.seconds() < armWaiting) {
            telemetry.addData("waiting for arm to get to position", "");
            telemetry.update();
            idle();
        }
        clampServo.setPower(0);
        clampServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void gyroTurning (double degrees) {

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (degrees-gyro.getHeading() > 180) {
            leftMotor.setPower(-0.5);
            rightMotor.setPower(0.5);
        } else {
            leftMotor.setPower(0.5);
            rightMotor.setPower(-0.5);
        }

        while(gyro.getHeading() != degrees) {

        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
