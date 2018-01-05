package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by justi on 2018-01-03.
 */

public abstract class NewAutonomous extends LinearOpMode {
    NewAutonomousDriver robot;
    int trackableViewed;

    /**
     * the high-level autonomous code using the methods in NewAutonomousDriver
     * use the red() and nearRelic() methods to determine the robot's starting position
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        scoreJewel();
        scoreGlyph();
    }

    /**
     * initializes the NewAutonomousDriver
     * this should not need to be edited
     */
    void initRobot() {
        robot = new NewAutonomousDriver(hardwareMap, this);
    }

    /**
     * lowers the jewel arm, reads the jewel colour, turns the robot to knock off the jewel,
     * turns back to starting angle, retracts the jewel arm
     */
    private void scoreJewel() {
        //TODO
    }

    /**
     * reads the vision pattern, drives off the balance board and aligns with the correct cryptobox
     * column, inserts the glyph, opens the clamp, and backs up slightly
     */
    private void scoreGlyph() {

    }

    /**
     * method overridden by child classes
     *
     * @return whether we are on the red team
     */
    abstract boolean red();

    /**
     * method overridden by child classes
     *
     * @return whether we are on the side near the relic recovery zone
     */
    abstract boolean nearRelic();
}
