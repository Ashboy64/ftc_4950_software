package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by justi on 2018-01-03.
 * this class is used by NewAutonomous for simpler movement and hardware control
 * implement the following methods as described in the comments
 */

public class NewAutonomousDriver {
    private LinearOpMode opMode;
    //TODO declaration of motors, servos, sensors, etc.

    public NewAutonomousDriver(HardwareMap hardwareMap, LinearOpMode opMode) {
        //TODO hardware initialization
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
            //TODO
        }
    }

    /**
     * opens the clamp by running the clamp servo until the open touch sensor is pressed
     */
    public void openClamp() {
        while (opMode.opModeIsActive())
        {
            //TODO
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