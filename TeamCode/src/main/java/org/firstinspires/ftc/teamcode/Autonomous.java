/*
AUTONOMOUS TODO
test gyro turning
test different field positions
test column detection with vuforia
read jewel colours and adjust thresholds
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public abstract class Autonomous extends LinearOpMode {
    private static final double TICKS_PER_MOTOR_REVOLUTION = 1120; //2240

    private static final double WHEEL_CIRCUMFERENCE_IN = 90 * 0.0393701 * Math.PI;
    private static final double DRIVE_GEAR_RATIO = 45.0 / 30;
    private static final double TURN_DISTANCE = 12.75 * Math.PI;

    private static final double GLYPH_OFFSET_RIGHT = 2.8515;
    private static final double GLYPH_OFFSET_FORWARD = 7.588 + 3;

    private long startTime;

    private static final double DRIVE_POWER = 1.0 / 8;
    private static final double TURN_POWER = 1.0 / 8;

    private static final double ENCODER_TURN_MULTIPLIER = 1.1;

    private static final int GYRO_TURN_THRESHOLD = 2;

    private CRServo clampServo;

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private GyroSensor gyro;

    private DigitalChannel armTouchOpen;
    private DigitalChannel armTouchClosed;

    private Servo jewelServo;
    private ColorSensor jewelColour;

    private VuforiaTrackable vuforiaTrackable;

    private RobotInput input;

    private int targetHeading = 0;

    private boolean useGyro = false;
    private boolean jewel = true;
    private boolean useVuforia = false;

    @Override
    public void runOpMode() {
        input = new RobotInput(gamepad1, gamepad2);

        initHardware();

        waitForStart();
        /*
        while (!isStarted() && !isStopRequested()) {
            sensorTelemetry();
        }
        */

        //movementTest();
        //sensorTest();
        startTime = System.currentTimeMillis();

        autonomous();
    }

    private void sensorTelemetry() {
        while (opModeIsActive()) {
            telemetry.addData("team", teamColour());
            telemetry.addData("near relic", nearRelic());
            telemetry.addData("jewel colour", jewelColour());
            telemetry.addData("column", targetColumn());
            if (useGyro) {
                telemetry.addData("gyro heading", gyroHeading());
            }
            telemetry.update();
        }
    }

    private void movementTest() {
        telemetry.addData("current task", "encoder drive test");
        telemetry.update();

        encoderDrive(24, DRIVE_POWER);
        sleep();
        encoderDrive(-24, DRIVE_POWER);
        sleep();

        if (useGyro) {
            telemetry.addData("current task", "gyro turn test");
            telemetry.update();

            gyroTurn(90, TURN_POWER);
            sleep();
            gyroTurn(-90, TURN_POWER);
            sleep();
        } else {
            telemetry.addData("current task", "encoder turn test");
            telemetry.update();

            encoderTurn(90, TURN_POWER);
            sleep();
            encoderTurn(-90, TURN_POWER);
            sleep();
        }

        telemetry.addData("current task", "open clamp");
        telemetry.update();

        openClamp();
        sleep();

        telemetry.addData("current task", "all tests complete");
        telemetry.update();

        while (opModeIsActive()) {
            sensorTelemetry();
        }
    }

    private void autonomous() {
        if (jewel) {
            jewel();
        }

        if (useVuforia) {
            int column = getColumn();
            glyph(column);
        } else {
            //glyph in centre column
            glyph(0);
        }

        telemetry.addData("current task", "autonomous complete");
        telemetry.update();

        while (opModeIsActive()) {
            sensorTelemetry();
        }
    }

    private void jewel() {
        telemetry.addData("current task", "jewel");
        telemetry.update();

        //drives slightly closer to jewel
        double adjustment = -1;

        //amount to turn to reread jewel colour
        int turnRead = 3;

        //degrees to turn to knock off jewel
        //positive is clockwise
        int turnDegrees = 25;

        //lower jewel arm
        setJewelArmPosition(0);
        sleep(2000);

        //drive closer
        encoderDrive(adjustment, DRIVE_POWER);
        sleep();

        //get jewel colour
        int jewelColour = jewelColour();

        //if uncertain, get closer and read again
        if (jewelColour == 0) {
            turn(turnRead, TURN_POWER);
            sleep();

            jewelColour = jewelColour();

            turn(-turnRead, TURN_POWER);
            sleep();
        }

        telemetry.addData("jewel colour", jewelColour);
        telemetry.update();

        //if certain about jewel colour
        if (jewelColour != 0) {
            if (jewelColour == teamColour()) {
                //our colour sensor faces right
                //facing our own jewel, turn ccw
                turnDegrees *= -1;
            }

            //turns to knock off the jewel
            turn(turnDegrees, TURN_POWER);
            sleep();

            //raise jewel arm
            setJewelArmPosition(1);

            //turns back
            turn(-turnDegrees, TURN_POWER);
            sleep();
        } else {
            //raise jewel arm, abandon attempt
            setJewelArmPosition(1);
        }

        //backs away to starting position
        encoderDrive(-adjustment, DRIVE_POWER);
        sleep();
    }

    private void sleep() {
        sleep(125);
    }

    private int getColumn() {
        telemetry.addData("current task", "get column");
        telemetry.update();

        int turn = -25;

        //turns toward vision target
        turn(turn, TURN_POWER);
        sleep();

        //gets column value
        int column = targetColumn();

        if (column < -1 || column > 1) {
            column = 0;
        }

        //turns back
        turn(-turn, TURN_POWER);
        sleep();

        return column;
    }

    private void glyph(int column) {
        telemetry.addData("current task", "score glyph");
        telemetry.update();

        //we drive a little farther because of dismounting the balancing stone
        double balanceDismountOffset = 4;

        //distance to drive to insert the glyph into the cryptobox
        double cryptoboxInsertion = 24 - GLYPH_OFFSET_FORWARD - 3;

        //distance to back away at the end
        double backAway = -2.5;

        //drives angled slightly away from cryptobox so we don't run into it
        //in case turning is not accurate, will still score glyph
        int outAngle = 5;

        //here we multiply by teamColour(), an integer representing -1 for blue or 1 for red
        //this reverses our turning direction if we are on the opposite alliance while
        //preventing unnecessary code duplication

        //if we are on the balancing stone closer to the relic scoring zone
        if (nearRelic()) {
            //turns to face cryptobox
            turn((90 - outAngle) * teamColour(), TURN_POWER);
            sleep();

            //drives off and aligns with correct column
            encoderDrive(36 + balanceDismountOffset + glyphHorizontalOffset(column), DRIVE_POWER);
            sleep();

            //turns to face correct column
            turn((90 + outAngle) * teamColour(), TURN_POWER);
            sleep();
        } else {
            //turns to dismount balancing stone
            turn(90 * teamColour(), TURN_POWER);
            sleep();

            //dismounts balancing stone
            encoderDrive(24 + balanceDismountOffset, DRIVE_POWER);
            sleep();

            //turns parallel to cryptobox
            turn((-90 - outAngle) * teamColour(), TURN_POWER);
            sleep();

            //aligns with correct column
            encoderDrive(glyphHorizontalOffset(column) + 12, DRIVE_POWER);
            sleep();

            //turns to face cryptobox
            turn((90 + outAngle) * teamColour(), TURN_POWER);
            sleep();
        }

        //releases glyph
        openClamp();
        sleep();

        //inserts glyph, with timeout so that we back away before the 30 second time limit
        encoderDrive(cryptoboxInsertion, DRIVE_POWER, 27000);
        sleep();

        //backs away from glyph so we aren't touching it
        encoderDrive(backAway, DRIVE_POWER);
        sleep();
    }

    private double glyphHorizontalOffset(int column) {
        double columnOffset = 7.5;

        //red means left is farther (approach from right)
        //blue means right is farther (approach from left)

        double offset = column * columnOffset * teamColour() * -1;

        //if approaching from right, need to drive farther
        //otherwise, need to drive less

        offset += GLYPH_OFFSET_RIGHT * teamColour();

        return offset;
    }

    private void initHardware() {
        telemetry.addData("hardware initialization", "working");
        telemetry.update();

        clampServo = hardwareMap.crservo.get("clampServo");

        armTouchOpen = hardwareMap.get(DigitalChannel.class, "tsOpen");
        armTouchClosed = hardwareMap.get(DigitalChannel.class, "tsClosed");
        armTouchOpen.setMode(DigitalChannel.Mode.INPUT);
        armTouchClosed.setMode(DigitalChannel.Mode.INPUT);

        if (useGyro) {
            gyro = hardwareMap.gyroSensor.get("gyro");
        }


        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motorZeroPowerBrake(true);

        jewelServo = hardwareMap.servo.get("jewelServo");
        jewelColour = hardwareMap.colorSensor.get("jewelColor");

        initVuforia();
        if (useGyro) {
            gyroCalibrate();
        }

        telemetry.addData("hardware initialization", "complete");
        telemetry.update();
    }

    private void motorZeroPowerBrake(boolean brake) {
        for (DcMotor motor : new DcMotor[]{leftMotor, rightMotor}) {
            if (brake) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }
    }

    public void freeDrive(double left, double right) {
        telemetry.addData("freedrive", "left %.4f right %.4f", left, right);
        telemetry.update();

        driveWithEncoders(false);
        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }

    private void stopDrive() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private void gyroTurn(int angle, double power) {
        int current = angleNormalize(gyroHeading());
        targetHeading = angleNormalize(targetHeading + angle);
        int delta = angleNormalize(targetHeading - current);

        telemetry.addData("gyro turn", "current %d, target %d, delta %d", current, targetHeading, delta);
        //telemetry.update();

        int direction = delta > 0 ? 1 : -1;

        freeDrive(direction * power, -direction * power);

        telemetry.addData("gyro turn", "power %.4f direction %d", power, direction);
        telemetry.update();

        long time = System.currentTimeMillis();
        while (opModeIsActive() && Math.abs(angleNormalize(gyroHeading() - targetHeading)) > GYRO_TURN_THRESHOLD) {
            //telemetry.addData("gyro turn", "angle %d, %d ms", gyroHeading(), System.currentTimeMillis() - time);
            //telemetry.update();
        }
        stopDrive();
    }

    private void encoderTurn(int angle, double power) {
        double robotRotations = angle / 360.0;
        double distance = robotRotations * TURN_DISTANCE * ENCODER_TURN_MULTIPLIER;

        telemetry.addData("encoder turn", "angle %d distance %.4f", angle, distance);
        telemetry.update();

        encoderDrive(distance, -distance, power, 99999999);
    }

    private void turn(int angle, double power) {
        if (useGyro) {
            gyroTurn(angle, power);
        } else {
            encoderTurn(angle, power);
        }
    }

    private void encoderDrive(double distance, double power) {
        encoderDrive(distance, distance, power, 99999999);
    }

    private void encoderDrive(double distance, double power, int timeout) {
        encoderDrive(distance, distance, power, timeout);
    }

    private void encoderDrive(double leftInches, double rightInches, double power, int timeout) {
        double leftRotations = leftInches / WHEEL_CIRCUMFERENCE_IN;
        double rightRotations = rightInches / WHEEL_CIRCUMFERENCE_IN;

        double leftShaft = leftRotations / DRIVE_GEAR_RATIO;
        double rightShaft = rightRotations / DRIVE_GEAR_RATIO;

        int leftTicks = (int) Math.round(leftShaft * TICKS_PER_MOTOR_REVOLUTION);
        int rightTicks = (int) Math.round(rightShaft * TICKS_PER_MOTOR_REVOLUTION);

        encoderDriveTicks(leftTicks, rightTicks, power, timeout);
    }

    private void encoderDriveTicks(int left, int right, double power, int timeout) {
        stopDrive();

        if (left == 0 && right == 0) {
            return;
        }

        driveWithEncoders(true);

        leftMotor.setPower(power);
        rightMotor.setPower(power);

        leftMotor.setTargetPosition(left);
        rightMotor.setTargetPosition(right);

        telemetry.addData("encoder drive", "driving");
        telemetry.update();

        while (opModeIsActive() && (leftMotor.isBusy() || rightMotor.isBusy()) && System.currentTimeMillis() < startTime + timeout) {
            telemetry.addData("encoder drive", "left %d target %d, right %d target %d",
                    leftMotor.getCurrentPosition(), left, rightMotor.getCurrentPosition(), right);
            telemetry.update();
        }

        stopDrive();
        telemetry.addData("encoder drive", "complete");
        telemetry.update();
    }

    private void driveWithEncoders(boolean enabled) {
        if (enabled) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else {
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private boolean getTouchOpen() {
        return !armTouchOpen.getState();
    }

    public void openClamp() {
        telemetry.addData("clamp", "opening");
        telemetry.update();
        clampServo.setPower(-1);

        long time = System.currentTimeMillis();
        while (opModeIsActive() && !getTouchOpen()) {
            telemetry.addData("clamp", "opening %d ms", System.currentTimeMillis() - time);
            telemetry.update();
        }
        clampServo.setPower(0);
        telemetry.addData("clamp", "opened");
        telemetry.update();
        sleep();
    }

    private void gyroCalibrate() {
        telemetry.addData("gyro", "calibrating");
        telemetry.update();
        gyro.calibrate();

        long startTime = System.currentTimeMillis();
        while (!isStopRequested() && gyro.isCalibrating()) {
            telemetry.addData("gyro", "calibrating %d ms", System.currentTimeMillis() - startTime);
            telemetry.update();
            sleep(50);
            idle();
        }

        telemetry.addData("gyro", "calibrated");
        telemetry.update();
    }

    private int gyroHeading() {
        int heading = gyro.getHeading();
        return angleNormalize(heading);
    }

    private int angleNormalize(int angle) {
        while (angle < -180) {
            angle += 360;
        }
        while (angle >= 180) {
            angle -= 360;
        }

        return angle;
    }

    //returns -2 for unknown, -1 for left, 0 for centre, 1 for right
    private int targetColumn() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(vuforiaTrackable);
        telemetry.addData("vuforia", vuMark.toString());

        if (vuMark == RelicRecoveryVuMark.LEFT) {
            return -1;
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            return 0;
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            return 1;
        } else {
            return -2;
        }
    }

    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AWnZ5xz/////AAAAGYmbM16TXEdKscTtfaECY6FzIRnxfc6SV0uwUV+dwPVIWbGyu9567BTp2qzh6ohnawdFrbL290ECRr04ew/QX0Q90SUrGh52+s55yVFPN429A93YJm6AlnV/TEJKb8omxdlqC+Hfy0SLPZSu+UEq9xQMOIfeW+OiRNQyFlUTZNCtQDNuK5jwObgulF83zrexs+c95Cd1jU7PnoX+NgHPjmUWS5H+WVr4yZsewES+oa0jRjGrcGU0/P5USRnqVbKh4976SNjPBGy6fanxJZmQb2Pam56UROtERcdaPDSWg4Nrr0MFlHCvi3PcfyLfdPtBW06JZGWBXu23VJCBQFw3SxGm/IO057P4kbTFti3W5xkU";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables trackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        vuforiaTrackable = trackables.get(0);
        trackables.activate();
    }

    //return -1 for blue, 1 for red, 0 for unknown
    private int jewelColour() {
        int blueThreshold = -32; //-128;
        int redThreshold = 64; //0;

        int red = jewelColour.red();
        int green = jewelColour.green();
        int blue = jewelColour.blue();

        int colourRaw = red - blue;

        telemetry.addData("colour", "%d %d %d (red - blue = %d)", red, green, blue, colourRaw);

        if (colourRaw < blueThreshold) {
            return -1;
        } else if (colourRaw > redThreshold) {
            return 1;
        } else {
            return 0;
        }
    }

    private void setJewelArmPosition(double position) {
        jewelServo.setPosition(position);
    }

    //return -1 for blue, 1 for red
    abstract int teamColour();

    abstract boolean nearRelic();
}