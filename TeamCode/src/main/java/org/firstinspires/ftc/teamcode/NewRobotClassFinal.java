package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

/**
 * Created by rao_a on 1/9/2018.
 */

public class NewRobotClassFinal {
    double wheel_diameter = 3.54331; //the diameter of the wheels of our robot.
    double wheel_circumference = Math.PI * wheel_diameter; //the value of π times the wheel diameter
    int ticksPerRevolution = (1120 * 60) / 96; //the amount of ticks the encoder takes to revolve one wheel
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor armMotor;
    ColorSensor colorSensor;
    GyroSensor gyro;
    VuforiaLocalizer vuforia;
    CRServo clampServo;
    Servo jewelServo;
    DigitalChannel ARM_TOUCH_OPEN;
    DigitalChannel ARM_TOUCH_CLOSED;
    LinearOpMode opMode;


    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public NewRobotClassFinal() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode opMode) {
        this.opMode = opMode;
        // save reference to HW Map
        hwMap = ahwMap;
        leftMotor = ahwMap.dcMotor.get("leftMotor");
        rightMotor = ahwMap.dcMotor.get("rightMotor");
        armMotor = ahwMap.dcMotor.get("armMotor");
        gyro = ahwMap.gyroSensor.get("gyro");
        clampServo = ahwMap.crservo.get("clampServo");
        colorSensor = ahwMap.colorSensor.get("jewelColor");
        jewelServo = ahwMap.servo.get("jewelServo");
        ARM_TOUCH_OPEN = ahwMap.get(DigitalChannel.class, "tsOpen");
        ARM_TOUCH_CLOSED = ahwMap.get(DigitalChannel.class, "tsClosed");
        ARM_TOUCH_OPEN.setMode(DigitalChannel.Mode.INPUT);
        ARM_TOUCH_CLOSED.setMode(DigitalChannel.Mode.INPUT);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro.calibrate();

        while (gyro.isCalibrating()) {

        }
    }


    public void gyroEncoders (double degrees) {
        int ticks = ticksPerRevolution;

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition((int)degrees*ticks);
        rightMotor.setTargetPosition((int)-degrees*ticks);

        if(degrees > 0) {
            leftMotor.setPower(0.5);
            rightMotor.setPower(-0.5);
        }else{
            leftMotor.setPower(-0.5);
            rightMotor.setPower(0.5);
        }

        while ((leftMotor.getCurrentPosition() < leftMotor.getTargetPosition()) && (rightMotor.getCurrentPosition() > rightMotor.getTargetPosition()) && (opMode.opModeIsActive())) {
            opMode.telemetry.addData(">" , gyro.getHeading());
            opMode.telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void setJewelArmPosition(double position) {
        jewelServo.setPosition(position);

        try {
            wait(1000);
        } catch (InterruptedException e) {
            opMode.telemetry.addData("biggie killed tupac > ", "1");
            opMode.telemetry.update();
        }

    }

    public void drive(double distance, LinearOpMode linearOpMode) {
        int encoderTicks = (int) ((distance / wheel_circumference) * ticksPerRevolution);

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

    public void openClamp(LinearOpMode linearOpMode) {
        clampServo.setDirection(DcMotorSimple.Direction.REVERSE);
        clampServo.setPower(1);
        while (getTouchOpen() && linearOpMode.opModeIsActive()) {

        }
        clampServo.setPower(0);
        clampServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void gyroTurning(int degrees) {
        if(degrees == 0) return;
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double currentHeading = gyro.getHeading();
        double target = (currentHeading + degrees) % 360;

        while (opMode.opModeIsActive() && !((gyro.getHeading()<degrees+5) && (gyro.getHeading()>degrees-5))) {
            double speed = 0.3 + (0.2 * Math.abs((gyro.getHeading() - target) / degrees));
            rightMotor.setPower((degrees < 0 ? speed : -speed));
            leftMotor.setPower((degrees < 0 ? -speed : speed));
            opMode.telemetry.addData("Heading:", gyro.getHeading());
            opMode.telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        opMode.telemetry.addData("Final heading", gyro.getHeading());
        opMode.telemetry.update();
    }

    public boolean getTouchOpen() {
        return !ARM_TOUCH_OPEN.getState();
    }

    public int getJewelColour(){
        //setJewelArmPosition(1);
        //wait(1000);
        opMode.telemetry.addData("in getJewelColour", "");
        opMode.telemetry.update();
        double blueColor = colorSensor.blue();
        double redColor = colorSensor.red();
        opMode.telemetry.addData("exiting getJewelColour", "");
        opMode.telemetry.update();
        return (blueColor > redColor ? -1 : 1);
    }

}
