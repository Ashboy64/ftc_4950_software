package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class Autonomous extends LinearOpMode {
    private final boolean TESTING = true;
    private final double DRIVE_POWER = 0.25;
    private final double TURN_POWER = 0.25;
    private RobotHardware HARDWARE;
    private RobotInput INPUT;

    @Override
    public void runOpMode() throws InterruptedException {
        HARDWARE = new RobotHardware(hardwareMap, this);
        INPUT = new RobotInput(gamepad1, gamepad2);

        HARDWARE.motorZeroPowerBrake(true);

        if (TESTING) {
            testing();
        } else {
            autonomous();
        }
    }

    private void testing() {
        while (opModeIsActive()) {
            int turnTestLarge = 90;
            int turnTestSmall = 45;
            double driveTestLarge = 12;
            double driveTestSmall = 4;

            if (INPUT.GAMEPAD.x()) {
                if (INPUT.GAMEPAD.leftTrigger() > 0) {
                    HARDWARE.gyroTurn(-turnTestLarge, TURN_POWER);
                } else if (INPUT.GAMEPAD.rightTrigger() > 0) {
                    HARDWARE.gyroTurn(turnTestLarge, TURN_POWER);
                } else if (INPUT.GAMEPAD.leftBumper()) {
                    HARDWARE.gyroTurn(-turnTestSmall, TURN_POWER);
                } else if (INPUT.GAMEPAD.rightBumper()) {
                    HARDWARE.gyroTurn(turnTestSmall, TURN_POWER);
                }
            } else if (INPUT.GAMEPAD.y()) {
                if (INPUT.GAMEPAD.leftTrigger() > 0) {
                    HARDWARE.encoderDrive(-driveTestLarge, DRIVE_POWER);
                } else if (INPUT.GAMEPAD.rightTrigger() > 0) {
                    HARDWARE.encoderDrive(driveTestLarge, DRIVE_POWER);
                } else if (INPUT.GAMEPAD.leftBumper()) {
                    HARDWARE.encoderDrive(-driveTestSmall, DRIVE_POWER);
                } else if (INPUT.GAMEPAD.rightBumper()) {
                    HARDWARE.encoderDrive(driveTestSmall, DRIVE_POWER);
                }
            } else if (INPUT.GAMEPAD.b()) {
                if (INPUT.GAMEPAD.leftTrigger() > 0) {
                    HARDWARE.encoderTurn(-turnTestLarge, TURN_POWER);
                } else if (INPUT.GAMEPAD.rightTrigger() > 0) {
                    HARDWARE.encoderTurn(turnTestLarge, TURN_POWER);
                } else if (INPUT.GAMEPAD.leftBumper()) {
                    HARDWARE.encoderTurn(-turnTestSmall, TURN_POWER);
                } else if (INPUT.GAMEPAD.rightBumper()) {
                    HARDWARE.encoderTurn(turnTestSmall, TURN_POWER);
                }
            } else if (INPUT.GAMEPAD.a()) {
                HARDWARE.openClamp();
            } else if (INPUT.GAMEPAD.rightTrigger() > 0) {
                //HARDWARE.setJewelArmPosition(0);
            } else if (INPUT.GAMEPAD.leftTrigger() > 0) {
                //HARDWARE.setJewelArmPosition(1);
            }

            //telemetry.addLine("jewel colour: " + HARDWARE.jewelColour().toString());
            telemetry.addLine("vision pattern: " + HARDWARE.targetColumn().toString());
        }
    }

    private void autonomous() {
        double balanceBoardCompensation = 0.5;
        double glyphInsert = 24 - RobotHardware.GLYPH_OFFSET_FORWARD;
        double backAway = -4;

        //jewel();

        RobotHardware.TargetColumn targetColumn = getColumn();

        int turnDirection;
        if (teamColour() == RobotHardware.TeamColour.BLUE) {
            turnDirection = 1;
        } else {
            turnDirection = -1;
        }

        if (nearRelic()) {
            HARDWARE.encoderTurn(-90 * turnDirection, TURN_POWER);
            HARDWARE.encoderDrive(36 + columnCompensation(targetColumn) + balanceBoardCompensation, DRIVE_POWER);
        } else {
            HARDWARE.encoderTurn(-90 * turnDirection, TURN_POWER);
            HARDWARE.encoderDrive(24 + balanceBoardCompensation, DRIVE_POWER);
            HARDWARE.encoderTurn(90 * turnDirection, TURN_POWER);
            HARDWARE.encoderDrive(6 + columnCompensation(targetColumn), DRIVE_POWER);
        }

        HARDWARE.encoderTurn(-90 * turnDirection, TURN_POWER);
        HARDWARE.openClamp();
        HARDWARE.encoderDrive(glyphInsert, DRIVE_POWER);
        HARDWARE.encoderDrive(backAway, DRIVE_POWER);
    }

    private double columnCompensation(RobotHardware.TargetColumn column) {
        double columnOffset = 7.5;
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
        int armDown = 1;
        int armUp = 0;
        double forwardAdjustment = -0.5;
        int turnDegrees = 15;

        HARDWARE.setJewelArmPosition(armDown);
        HARDWARE.encoderDrive(forwardAdjustment, DRIVE_POWER);

        RobotHardware.TeamColour jewelColour = HARDWARE.jewelColour();

        if (jewelColour != RobotHardware.TeamColour.UNDEFINED) {
            if (jewelColour == teamColour()) {
                turnDegrees *= -1;
            }

            HARDWARE.gyroTurn(turnDegrees, TURN_POWER);
            HARDWARE.gyroTurn(-turnDegrees, TURN_POWER);
        }

        HARDWARE.encoderDrive(-forwardAdjustment, DRIVE_POWER);
        HARDWARE.setJewelArmPosition(armUp);
    }

    private RobotHardware.TargetColumn getColumn() {
        HARDWARE.initVuforia();

        int turns = 0;
        int turnDegrees = -5;
        RobotHardware.TargetColumn column = RobotHardware.TargetColumn.UNDEFINED;

        while (true) {
            column = HARDWARE.targetColumn();

            if (column == RobotHardware.TargetColumn.UNDEFINED) {
                turns++;
                HARDWARE.encoderTurn(turnDegrees, TURN_POWER);
            } else {
                int turnBack = turnDegrees * turns * -1;
                HARDWARE.encoderTurn(turnBack, TURN_POWER);
                return column;
            }
        }
    }

    abstract RobotHardware.TeamColour teamColour();

    abstract boolean nearRelic();
}
