package org.firstinspires.ftc.teamcode;

/**
 * Created by rao_a on 11/30/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


public class RobotClassFinalUse {
    double wheel_diameter = 3.54331; //the diameter of the wheels of our robot.
    double wheel_circumference = Math.PI * wheel_diameter; //the value of π times the wheel diameter
    int ticksPerRevolution = (1120 * 60)/96; //the amount of ticks the encoder takes to revolve one wheel
    double armWaiting = 2.0;
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor armMotor;
    ColorSensor colorSensor;
    GyroSensor gyro;
    VuforiaLocalizer vuforia;
    CRServo clampServo;
    //CRServo jewelServo;
    int version;


    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    public RobotClassFinalUse() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;
        leftMotor = ahwMap.dcMotor.get("leftMotor");
        rightMotor = ahwMap.dcMotor.get("rightMotor");
        armMotor = ahwMap.dcMotor.get("armMotor");
        gyro = ahwMap.gyroSensor.get("gyro");
        clampServo = ahwMap.crservo.get("clampServo");
        //colorSensor = ahwMap.colorSensor.get("colorSensor");
        //jewelServo = ahwMap.crservo.get("jewelServo");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro.calibrate();

        while(gyro.isCalibrating()){

        }
    }

    public void movingForward(double distance, LinearOpMode linearOpMode) {
        int encoderTicks = (int) ((distance/wheel_circumference) * ticksPerRevolution);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition(encoderTicks);
        rightMotor.setTargetPosition(encoderTicks);

        leftMotor.setPower(1);
        rightMotor.setPower(1);

        while (leftMotor.getCurrentPosition() < leftMotor.getTargetPosition()
                && rightMotor.getCurrentPosition() < rightMotor.getTargetPosition()
                && linearOpMode.opModeIsActive()) {

        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void armMoving(LinearOpMode linearOpMode) {
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(1120);
        armMotor.setPower(1);
        while (armMotor.getCurrentPosition() < armMotor.getTargetPosition() && linearOpMode.opModeIsActive()) {

        }
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armGrabbing(linearOpMode);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(1120);
        armMotor.setPower(1);
    }

    public void armGrabbing(LinearOpMode linearOpMode) {
        ElapsedTime opmodeRunTime = new ElapsedTime();
        clampServo.setPower(1);
        while (opmodeRunTime.seconds() < armWaiting && linearOpMode.opModeIsActive()) {

        }
        clampServo.setPower(0);
    }

    public void armRelease(LinearOpMode linearOpMode) {
        ElapsedTime opmodeRunTime = new ElapsedTime();
        clampServo.setDirection(DcMotorSimple.Direction.REVERSE);
        clampServo.setPower(1);
        while (opmodeRunTime.seconds() < armWaiting && linearOpMode.opModeIsActive()) {

        }
        clampServo.setPower(0);
        clampServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void gyroTurning (double degrees, LinearOpMode linearOpMode) {
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double target = (gyro.getHeading() + degrees)%360;
        double range = 5.00001;
        while(gyro.getHeading() > target + range || gyro.getHeading() < target - range && linearOpMode.opModeIsActive()){
            if (gyro.getHeading() > target) {
                leftMotor.setPower(-0.5);
                rightMotor.setPower(0.5);
            }else if (gyro.getHeading() < target) {
                leftMotor.setPower(0.5);
                rightMotor.setPower(-0.5);
            }

            if(linearOpMode.gamepad1.b){
                break;
            }

        }



        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}