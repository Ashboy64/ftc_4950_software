package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public interface RadianGyro {
    /**
     * Defines the orientation of the gyro.
     */
    enum Orientation {
        /**
         * Normal orientation of the gyro (facing up).
         */

        NORMAL(1),
        /**
         * Reversed orientation (upside-down).
         */
        REVERSE(-1);

        /**
         * Value to multiply unconverted heading value by to return heading value corrected for
         * gyro orientation.
         */
        final int SCALAR;

        Orientation(int scalar) {
            SCALAR = scalar;
        }
    }

    /**
     * Returns the gyro orientation.
     * @return the gyro orientation
     */
    Orientation getOrientation();

    /**
     * Sets the orientation of the gyro. Set to REVERSE if the gyro is mounted upside-down.
     * @param orientation the orientation of the gyro
     */
    void setOrientation(Orientation orientation);

    /**
     * Returns the gyro heading without accounting for orientation.
     * @return the gyro heading in radians
     */
    double getHeadingUnconverted();

    /**
     * Returns the gyro heading.
     * @return the gyro heading in radians
     */
    double getHeading();

    /**
     * Returns the unit circle position of the gyro angle.
     * @return the unit circle position of the gyro angle
     */
    double getUnitCircle();
}

/**
 * A wrapper class for the Modern Robotics gyro and RevRobotics Expansion Hub IMU.
 */
abstract class RadianGyroSensor implements RadianGyro {
    private Orientation orientation = Orientation.NORMAL;

    /**
     * Starts gyro calibration
     */
    public abstract void startCalibrate();

    /**
     * Gets gyro calibration status.
     * @return whether the gyro is still calibrating
     */
    public abstract boolean isCalibrating();

    @Override
    public Orientation getOrientation() {
        return orientation;
    }

    @Override
    public void setOrientation(Orientation orientation) {
        this.orientation = orientation;
    }

    @Override
    public double getHeading() {
        return Utils.angleNormalize(getHeadingUnconverted() * getOrientation().SCALAR);
    }

    @Override
    public double getUnitCircle() {
        return Utils.toUnitCircle(getHeading());
    }
}

class MRRadianGyro extends RadianGyroSensor {
    private ModernRoboticsI2cGyro gyro;

    /**
     * Creates a RadianGyroSensor from a ModernRobotics gyro.
     * @param gyro the Modern Robotics gyro
     * @return a new RadianGyroSensor using the Modern Robotics gyro
     */
    public MRRadianGyro(ModernRoboticsI2cGyro gyro) {
        this.gyro = gyro;
    }

    @Override
    public void startCalibrate() {
        gyro.calibrate();
    }

    @Override
    public boolean isCalibrating() {
        return gyro.isCalibrating();
    }

    @Override
    public double getHeadingUnconverted() {
        return Math.toRadians(gyro.getHeading());
    }
}

class ImuRadianGyro extends RadianGyroSensor {
    private BNO055IMU imu;

    /**
     * Creates a RadianGyroSensor from the Expansion Hub IMU.
     * @param imu the Expansion Hub IMU
     * @return a new RadianGyroSensor using the Expansion Hub IMU
     */
    public ImuRadianGyro(BNO055IMU imu) {
        this.imu = imu;
    }

    @Override
    public void startCalibrate() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
    }

    @Override
    public boolean isCalibrating() {
        return false;
    }

    @Override
    public double getHeadingUnconverted() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }
}