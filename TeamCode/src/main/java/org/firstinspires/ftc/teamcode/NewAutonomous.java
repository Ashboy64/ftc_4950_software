package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by justin on 2018-01-03.
 */

public abstract class NewAutonomous extends LinearOpMode {
    NewAutonomousDriver robot;
    int trackableViewed;
    double cryptoBoxWidth = 7.63;
    double armOffset = 2.8515;

    /**
     * the high-level autonomous code using the methods in NewAutonomousDriver
     * use the red() and nearRelic() methods to determine the robot's starting position
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        //scoreJewel();
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
    private void scoreJewel() throws InterruptedException{
        int detected = robot.getJewelColour();
        robot.gyroEncoders((red() ? (detected == -1 ? -90 : 90) : (detected == 1 ? 90 : -90)));
    }

    /**
     * reads the vision pattern, drives off the balance board and aligns with the correct cryptobox
     * column, inserts the glyph, opens the clamp, and backs up slightly
     */
    private void scoreGlyph() {
        if (!red() && !nearRelic()) {
            robot.turn(135);
            trackableViewed = robot.getTargetColumn();
            robot.turn(-135);
            robot.drive(((cryptoBoxWidth/2) + (trackableViewed * cryptoBoxWidth)) - armOffset);
            robot.turn(-90);
            robot.drive(18);
            robot.openClamp();
        } else if  (!red() && nearRelic()) {
            robot.turn(135);
            trackableViewed = robot.getTargetColumn();
            robot.turn(135);
            robot.drive(24);
            robot.drive(((cryptoBoxWidth/2) + (trackableViewed * cryptoBoxWidth)) - armOffset);
            robot.turn(-90);
            robot.drive(18);
            robot.openClamp();
        } else if (red() && nearRelic()) {
            robot.turn(135);
            trackableViewed = robot.getTargetColumn();
            robot.turn(-135);
            robot.drive(((cryptoBoxWidth/2) + (trackableViewed * cryptoBoxWidth)) - armOffset);
            robot.turn(90);
            robot.drive(18);
            robot.openClamp();
        } else {
            robot.turn(135);
            robot.turn(135);
            trackableViewed = robot.getTargetColumn();
            robot.turn(-45);
            robot.drive(24);
            robot.drive(((cryptoBoxWidth/2) + (trackableViewed * cryptoBoxWidth)) - armOffset);
            robot.turn(90);
            robot.drive(18);
            robot.openClamp();
        }
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
