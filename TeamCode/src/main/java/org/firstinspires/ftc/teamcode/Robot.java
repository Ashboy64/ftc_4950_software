package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    /**
     * The robot's wheel circumferences in inches
     */
    //90 mm wheel diameter -> convert to inches -> find circumference
    public static final double WHEEL_CIRCUMFERENCE_IN = 90 * 0.0393701 * Math.PI;

    /**
     * The number of encoder ticks per revolution of the drive motors
     */
    public static final double TICKS_PER_MOTOR_REVOLUTION = 1120;

    /**
     * The gear ratio between the drive motor and wheel shaft
     */
    //number of teeth on gear on drive motor shaft / number of teeth on gear on wheel shaft
    public static final double DRIVE_GEAR_RATIO = 45.0 / 30.0;

    /**
     * The distance between wheels on opposite sides of the robot
     */
    public static final double DRIVE_WIDTH = 12.625;

    //keep track of the current Robot instance so we can access it outside of opmodes
    private static Robot robot;

    /**
     * The left drive motor
     */
    public final DcMotor DRIVE_LEFT;

    /**
     * The right drive motor
     */
    public final DcMotor DRIVE_RIGHT;

    /**
     * A Modern Robotics gyro
     */
    public final RadianGyroSensor GYRO;

    /**
     * The Expansion Hub imu
     */
    public final RadianGyroSensor IMU;

    /**
     * Creates a new Robot from the hardware map.
     * @param hardwareMap the HardwareMap to use for hardware initialization
     */
    public Robot(HardwareMap hardwareMap) {
        DRIVE_LEFT = hardwareMap.dcMotor.get("leftMotor");
        DRIVE_RIGHT = hardwareMap.dcMotor.get("rightMotor");
        DRIVE_LEFT.setDirection(DcMotorSimple.Direction.FORWARD);
        DRIVE_RIGHT.setDirection(DcMotorSimple.Direction.REVERSE);
        DRIVE_LEFT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DRIVE_RIGHT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        GYRO = new MRRadianGyro((ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro"));

        IMU = new ImuRadianGyro(hardwareMap.get(BNO055IMU.class, "imu"));
        IMU.setOrientation(RadianGyroSensor.Orientation.REVERSE); //our gyro is mounted upside-down

        //initializes the static Robot instance for outside classes to use
        robot = this;
    }

    /**
     * Convert inches driven by robot to drive motor encoder ticks
     * @param inches the number of inches
     * @return the number of encoder ticks
     */
    public static int inchesToEncoderTicks(double inches) {
        double wheelRotations = inches / WHEEL_CIRCUMFERENCE_IN;
        double shaftRotations = wheelRotations / DRIVE_GEAR_RATIO;
        return (int) Math.round(shaftRotations * TICKS_PER_MOTOR_REVOLUTION);
    }

    /**
     * Convert encoder ticks on drive motor to inches driven
     * @param ticks the number of encoder ticks
     * @return the number of inches driven
     */
    public static double encoderTicksToInches(int ticks)
    {
        double motorRotations = (double)ticks / TICKS_PER_MOTOR_REVOLUTION;
        double wheelRotations = motorRotations * DRIVE_GEAR_RATIO;
        return wheelRotations * WHEEL_CIRCUMFERENCE_IN;
    }

    public static Robot getRobot() {
        return robot;
    }
}
