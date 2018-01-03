package org.firstinspires.ftc.teamcode;

/**
 * Created by rao_a on 11/30/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    DigitalChannel ARM_TOUCH_OPEN;
    DigitalChannel ARM_TOUCH_CLOSED;


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
        ARM_TOUCH_OPEN = ahwMap.get(DigitalChannel.class, "tsOpen");
        ARM_TOUCH_CLOSED = ahwMap.get(DigitalChannel.class, "tsClosed");
        ARM_TOUCH_OPEN.setMode(DigitalChannel.Mode.INPUT);
        ARM_TOUCH_CLOSED.setMode(DigitalChannel.Mode.INPUT);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);

        while (leftMotor.getCurrentPosition() < leftMotor.getTargetPosition() && rightMotor.getCurrentPosition() < rightMotor.getTargetPosition() && linearOpMode.opModeIsActive()) {

        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void armRelease(LinearOpMode linearOpMode) {
        clampServo.setDirection(DcMotorSimple.Direction.REVERSE);
        clampServo.setPower(1);
        while (getTouchOpen() && linearOpMode.opModeIsActive()) {

        }
        clampServo.setPower(0);
        clampServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void gyroTurning (double degrees, LinearOpMode linearOpMode) {
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double target = (gyro.getHeading() + degrees);
        double range = 5.00001;
        while((gyro.getHeading() > target + range || gyro.getHeading() < target - range) && linearOpMode.opModeIsActive()){
            if (gyro.getHeading() > target) {
                leftMotor.setPower(-0.5);
                rightMotor.setPower(0.5);
            }else if (gyro.getHeading() < target) {
                leftMotor.setPower(0.5);
                rightMotor.setPower(-0.5);
            }
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean getTouchOpen() {
        return !ARM_TOUCH_OPEN.getState();
    }

    public boolean getTouchClosed() {
        return !ARM_TOUCH_CLOSED.getState();
    }
}