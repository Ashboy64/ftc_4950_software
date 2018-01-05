package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by justi on 2018-01-03.
 * this class is used by NewAutonomous for simpler movement and hardware control
 * implement the following methods as described in the comments
 */

public class NewAutonomousDriver {
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
    LinearOpMode opMode;


    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();
    //TODO declaration of motors, servos, sensors, etc.

    public NewAutonomousDriver(HardwareMap hardwareMap, LinearOpMode opMode) {
        // save reference to HW Map
        this.opMode = opMode;
        hwMap = hardwareMap;
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        gyro = hardwareMap.gyroSensor.get("gyro");
        clampServo = hardwareMap.crservo.get("clampServo");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        //jewelServo = hardwareMap.crservo.get("jewelServo");
        ARM_TOUCH_OPEN = hardwareMap.get(DigitalChannel.class, "tsOpen");
        ARM_TOUCH_CLOSED = hardwareMap.get(DigitalChannel.class, "tsClosed");
        ARM_TOUCH_OPEN.setMode(DigitalChannel.Mode.INPUT);
        ARM_TOUCH_CLOSED.setMode(DigitalChannel.Mode.INPUT);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro.calibrate();

        while(gyro.isCalibrating()){

        }
    }

    /**
     * turns the robot with the gyro
     * @param degrees turns the robot by this angle; positive is clockwise, negative is counterclockwise
     */
    public void turn(int degrees) {
        while (opMode.opModeIsActive())
        {
            //TODO
        }
    }

    /**
     * drives the robot using encoders
     * @param inches drives this many inches; positive is forwards, negative is backwards
     */
    public void drive(double inches) {
        while (opMode.opModeIsActive())
        {
            int encoderTicks = (int) ((inches/wheel_circumference) * ticksPerRevolution);

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

            while (leftMotor.getCurrentPosition() < leftMotor.getTargetPosition() && rightMotor.getCurrentPosition() < rightMotor.getTargetPosition() /* && opMode.opModeIsActive()*/) {

            }

            leftMotor.setPower(0);
            rightMotor.setPower(0);

            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /**
     * opens the clamp by running the clamp servo until the open touch sensor is pressed
     */
    public void openClamp() {
        while (opMode.opModeIsActive())
        {
            clampServo.setDirection(DcMotorSimple.Direction.REVERSE);
            clampServo.setPower(1);
            while (!ARM_TOUCH_OPEN.getState() && opMode.opModeIsActive()) {

            }
            clampServo.setPower(0);
            clampServo.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    /**
     * sets the jewel arm position
     * @param position the position to set the jewel arm to; 0 is retracted, 1 is lowered
     */
    public void setJewelArmPosition(double position) {
        //TODO
    }

    /**
     * reads the colour of the jewel from the colour sensor
     * @return -1 for a blue jewel, 1 for a red jewel
     */
    public int getJewelColour() {
        //TODO
        return 0;
    }

    /**
     * uses vuforia to read the vision target to determine which column to place the glyph in
     * @return -1 for the left column, 0 for the centre column, 1 for the right column, 2 for unknown
     *
     */
    public int getTargetColumn() {
        //TODO
        return 0;
    }
}