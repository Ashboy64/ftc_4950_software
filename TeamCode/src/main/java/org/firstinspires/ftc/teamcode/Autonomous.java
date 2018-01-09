package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
TESTING CONTROLS - hold Back while starting autonomous to enter testing mode
x: test jewel code
y: get column using vuforia
a: log jewel colour and vuforia vision target
b: open clamp
dpad:
    up: forward 4 inches
    down: backward 4 inches
    left: turn counterclockwise 15 degrees (gyro)
    right: turn clockwise 15 degrees (gyro)
trigger:
    left: turn counterclockwise 15 degrees (encoder)
    right: turn clockwise 15 degrees (encoder)
*/

public abstract class Autonomous extends LinearOpMode {
    private boolean testing = false;
    private final double DRIVE_POWER = 0.25;
    private final double TURN_POWER = 0.25;
    private RobotHardware HARDWARE;
    private RobotInput INPUT;

    @Override
    public void runOpMode() throws InterruptedException {
        HARDWARE = new RobotHardware(hardwareMap, this);
        INPUT = new RobotInput(gamepad1, gamepad2);

        HARDWARE.motorZeroPowerBrake(true);
        HARDWARE.gyroCalibrate();

        testing = INPUT.GAMEPAD.back();

        if (testing) {
            testing();
        } else {
            autonomous();
        }
    }

    private void testing() {
        while (opModeIsActive()) {
            if (INPUT.GAMEPAD.x()) {
                jewel();
            } else if (INPUT.GAMEPAD.y()) {
                telemetry.addData("column detected", getColumn());
            } else if (INPUT.GAMEPAD.a()) {
                telemetry.addData("jewel colour", HARDWARE.jewelColour());
                telemetry.addData("vision pattern", HARDWARE.targetColumn());
            } else if (INPUT.GAMEPAD.b()) {
                HARDWARE.openClamp();
            } else if (INPUT.GAMEPAD.dpadUp()) {
                telemetry.addLine("driving forward");
                HARDWARE.encoderDrive(4, DRIVE_POWER);
            } else if (INPUT.GAMEPAD.dpadDown()) {
                telemetry.addLine("driving backward");
                HARDWARE.encoderDrive(-4, DRIVE_POWER);
            } else if (INPUT.GAMEPAD.dpadRight()) {
                telemetry.addLine("turning right (gyro)");
                HARDWARE.gyroTurn(15, TURN_POWER);
            } else if (INPUT.GAMEPAD.dpadLeft()) {
                telemetry.addLine("turning left (gyro)");
                HARDWARE.gyroTurn(-15, TURN_POWER);
            } else if (INPUT.GAMEPAD.rightBumper()) {
                telemetry.addLine("turning right (encoder)");
                HARDWARE.encoderTurn(15, TURN_POWER);
            } else if (INPUT.GAMEPAD.leftBumper()) {
                telemetry.addLine("turning left (encoder)");
                HARDWARE.encoderTurn(-15, TURN_POWER);
            }
        }
    }

    private void autonomous() {
        jewel();

        RobotHardware.TargetColumn targetColumn = getColumn();

        glyph(targetColumn);
    }

    private void glyph(RobotHardware.TargetColumn targetColumn) {
        //if given no column, go for the centre
        if (targetColumn == RobotHardware.TargetColumn.UNDEFINED) {
            targetColumn = RobotHardware.TargetColumn.CENTRE;
        }

        double balanceBoardCompensation = 0.5;
        double glyphInsert = 24 - RobotHardware.GLYPH_OFFSET_FORWARD;
        double backAway = -4;

        int turnDirection;
        if (teamColour() == RobotHardware.TeamColour.BLUE) {
            turnDirection = 1;
        } else {
            turnDirection = -1;
        }

        if (nearRelic()) {
            HARDWARE.turn(-90 * turnDirection, TURN_POWER);
            HARDWARE.encoderDrive(36 + columnCompensation(targetColumn) + balanceBoardCompensation, DRIVE_POWER);
        } else {
            HARDWARE.turn(-90 * turnDirection, TURN_POWER);
            HARDWARE.encoderDrive(24 + balanceBoardCompensation, DRIVE_POWER);
            HARDWARE.turn(90 * turnDirection, TURN_POWER);
            HARDWARE.encoderDrive(6 + columnCompensation(targetColumn), DRIVE_POWER);
        }

        HARDWARE.turn(-90 * turnDirection, TURN_POWER);
        HARDWARE.openClamp();
        HARDWARE.encoderDrive(glyphInsert, DRIVE_POWER);
        HARDWARE.encoderDrive(backAway, DRIVE_POWER);
    }

    private double columnCompensation(RobotHardware.TargetColumn column) {
        double columnOffset = 7.5; //distance between cryptobox columns in inches
        double compensation = 0;

        if (nearRelic()) {
            if (teamColour() == RobotHardware.TeamColour.BLUE) {
                compensation -= RobotHardware.GLYPH_OFFSET_RIGHT;
                if (column == RobotHardware.TargetColumn.LEFT) {
                    compensation -= columnOffset;
                } else if (column == RobotHardware.TargetColumn.RIGHT) {
                    compensation += columnOffset;
                }
            } else {
                compensation += RobotHardware.GLYPH_OFFSET_RIGHT;
                if (column == RobotHardware.TargetColumn.LEFT) {
                    compensation += columnOffset;
                } else if (column == RobotHardware.TargetColumn.RIGHT) {
                    compensation -= columnOffset;
                }
            }
        } else {
            if (teamColour() == RobotHardware.TeamColour.BLUE) {
                compensation -= RobotHardware.GLYPH_OFFSET_RIGHT;
                if (column == RobotHardware.TargetColumn.LEFT) {
                    compensation -= columnOffset;
                } else if (column == RobotHardware.TargetColumn.RIGHT) {
                    compensation += columnOffset;
                }
            } else {
                compensation += RobotHardware.GLYPH_OFFSET_RIGHT;
                if (column == RobotHardware.TargetColumn.LEFT) {
                    compensation += columnOffset;
                } else if (column == RobotHardware.TargetColumn.RIGHT) {
                    compensation -= columnOffset;
                }
            }
        }

        return compensation;
    }

    private void jewel() {
        double adjustment = 1; //positive value backs away from jewel
        int turnDegrees = 15; //turns clockwise

        //backs away, lowers arm, moves forward (prevents arm hitting field wall)
        HARDWARE.encoderDrive(adjustment, DRIVE_POWER);
        HARDWARE.setJewelArmPosition(1);
        HARDWARE.encoderDrive(-adjustment, DRIVE_POWER);

        RobotHardware.TeamColour jewelColour = HARDWARE.jewelColour();
        telemetry.addData("jewel colour", jewelColour);

        if (jewelColour != RobotHardware.TeamColour.UNDEFINED) {
            //our colour sensor faces left (counterclockwise)
            //if we are facing our own jewel, we turn clockwise
            if (jewelColour != teamColour()) {
                //not facing our jewel, turn counterclockwise
                turnDegrees *= -1;
            }

            //turns to knock off the jewel, then turns back
            HARDWARE.gyroTurn(turnDegrees, TURN_POWER);
            HARDWARE.gyroTurn(-turnDegrees, TURN_POWER);
        }

        //backs away, raises arm, moves forward
        HARDWARE.encoderDrive(adjustment, DRIVE_POWER);
        HARDWARE.setJewelArmPosition(0);
        HARDWARE.encoderDrive(-adjustment, DRIVE_POWER);
    }

    private RobotHardware.TargetColumn getColumn() {
        int totalTurned = 0;

        //vision target mounted to the left when facing toward the jewels
        //turn counterclockwise if vision target not visible
        int turnDegrees = -5;

        //maximum number of turns before giving up on vision target and turning back
        int maxTurns = 10;

        RobotHardware.TargetColumn column = RobotHardware.TargetColumn.UNDEFINED;

        while (totalTurned / turnDegrees < maxTurns) {
            //get vision target
            column = HARDWARE.targetColumn();

            if (column == RobotHardware.TargetColumn.UNDEFINED) {
                //vision target not visible, turn clockwise
                totalTurned += turnDegrees;
                HARDWARE.turn(turnDegrees, TURN_POWER);
            } else {
                //vision target detected, stop turning
                break;
            }
        }

        //turn back and return detected vision target
        HARDWARE.turn(-totalTurned, TURN_POWER);
        return column;
    }

    abstract RobotHardware.TeamColour teamColour();

    abstract boolean nearRelic();
}
