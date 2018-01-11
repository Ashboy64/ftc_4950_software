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
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public abstract class Autonomous extends LinearOpMode {
    private static final int JEWEL_COLOUR_THRESHOLD = 64;

    private static final double TICKS_PER_MOTOR_REVOLUTION = 1120; //2240

    private static final double ARM_SPROCKET_RATIO = 15.0 / 54;
    private static final double WHEEL_CIRCUMFERENCE_IN = 90 * 0.0393701 * Math.PI;
    private static final double DRIVE_GEAR_RATIO = 45.0 / 30;
    private static final double TURN_DISTANCE = 12.75 * Math.PI;

    private static final double GLYPH_OFFSET_RIGHT = 2.8515;
    private static final double GLYPH_OFFSET_FORWARD = 7.588;

    private static final double DRIVE_POWER = 1.0 / 16;
    private static final double TURN_POWER = 1.0 / 16;

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

    @Override
    public void runOpMode() {
        input = new RobotInput(gamepad1, gamepad2);

        initHardware();
        waitForStart();

        autonomous();
    }

    private void autonomous() {
        telemetry.addData("current task", "jewel");
        telemetry.update();
        jewel();

        telemetry.addData("current task", "get column");
        telemetry.update();
        int column = getColumn();

        telemetry.addData("current task", "score glyph");
        telemetry.update();
        glyph(column);
    }

    private void jewel() {
        double adjustment = 1; //positive value backs away from jewel
        int turnDegrees = 15; //turns clockwise

        //backs away, lowers arm, moves forward (prevents arm hitting field wall)
        encoderDrive(adjustment, DRIVE_POWER);
        sleep();
        setJewelArmPosition(1);
        sleep();
        encoderDrive(-adjustment, DRIVE_POWER);
        sleep();

        int jewelColour = jewelColour();
        telemetry.addData("jewel colour", jewelColour);

        if (jewelColour != 0) {
            //our colour sensor faces left (counterclockwise)
            //if we are facing our own jewel, we turn clockwise
            if (jewelColour != teamColour()) {
                //not facing our jewel, turn counterclockwise
                turnDegrees *= -1;
            }

            //turns to knock off the jewel, then turns back
            turn(turnDegrees, TURN_POWER);
            sleep();
            turn(-turnDegrees, TURN_POWER);
            sleep();
        }

        //backs away, raises arm, moves forward
        encoderDrive(adjustment, DRIVE_POWER);
        sleep();
        setJewelArmPosition(0);
        sleep();
        encoderDrive(-adjustment, DRIVE_POWER);
        sleep();
    }

    private void sleep() {
        sleep(500);
    }

    private int getColumn() {
        int turn = -25;

        //turns toward vision target
        turn(turn, TURN_POWER);
        sleep();

        //gets column value
        int column = targetColumn();

        if (column < -1 || column > 1)
        {
            column = 0;
        }

        //turns back
        turn(-turn, TURN_POWER);
        sleep();

        return column;
    }

    private void glyph(int column) {
        double balanceDismountOffset = 0;
        double cryptoboxInsertion = 24 - GLYPH_OFFSET_FORWARD;
        double backAway = -2;

        if (nearRelic()) {
            turn(90 * teamColour(), TURN_POWER);
            sleep();
            encoderDrive(36 + balanceDismountOffset + glyphHorizontalOffset(column), DRIVE_POWER);
            sleep();
            turn(90 * teamColour(), TURN_POWER);
            sleep();
        } else {
            turn(90 * teamColour(), TURN_POWER);
            sleep();
            encoderDrive(24 + balanceDismountOffset, DRIVE_POWER);
            sleep();
            turn(-90 * teamColour(), TURN_POWER);
            sleep();
            encoderDrive(glyphHorizontalOffset(column), DRIVE_POWER);
            sleep();
            turn(90 * teamColour(), TURN_POWER);
            sleep();
        }

        //insert glyph

        openClamp();
        sleep();
        encoderDrive(cryptoboxInsertion, DRIVE_POWER);
        sleep();
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

        gyro = hardwareMap.gyroSensor.get("gyro");

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motorZeroPowerBrake(true);

        jewelServo = hardwareMap.servo.get("jewelServo");
        jewelColour = hardwareMap.colorSensor.get("jewelColor");

        initVuforia();
        gyroCalibrate();

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

    private void freeDrive(double left, double right) {
        driveWithEncoders(false);
        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }

    private void stopDrive() {
        freeDrive(0, 0);
    }

    private void gyroTurn(int angle, double power) {
        int current = angleNormalize(gyroHeading());
        int target = angleNormalize(current + angle);
        int delta = angleNormalize(target - current);

        telemetry.addData("gyro turn", "current %d, target %d, delta %d", current, target, delta);
        telemetry.update();

        int direction = delta > 0 ? 1 : -1;

        while (opModeIsActive() && angleNormalize(gyroHeading() - target) > GYRO_TURN_THRESHOLD) {
            freeDrive(direction * power, -direction * power);
            telemetry.addData("angle", gyroHeading());
            telemetry.update();
        }
        stopDrive();
    }

    private void encoderTurn(int angle, double power) {
        double robotRotations = angle / 360.0;
        double distance = robotRotations * TURN_DISTANCE;

        telemetry.addData("encoder turn", "angle %d distance %d", angle, distance);
        telemetry.update();

        encoderDrive(distance, -distance, power);
    }

    private void turn(int angle, double power) {
        gyroTurn(angle, power);
        //encoderTurn(angle, power);
    }

    private void encoderDrive(double distance, double power) {
        encoderDrive(distance, distance, power);
    }

    private void encoderDrive(double leftInches, double rightInches, double power) {
        double leftRotations = leftInches / WHEEL_CIRCUMFERENCE_IN;
        double rightRotations = rightInches / WHEEL_CIRCUMFERENCE_IN;

        double leftShaft = leftRotations / DRIVE_GEAR_RATIO;
        double rightShaft = rightRotations / DRIVE_GEAR_RATIO;

        int leftTicks = (int) Math.round(leftShaft * TICKS_PER_MOTOR_REVOLUTION);
        int rightTicks = (int) Math.round(rightShaft * TICKS_PER_MOTOR_REVOLUTION);

        encoderDriveTicks(leftTicks, rightTicks, power);
    }

    private void encoderDriveTicks(int left, int right, double power) {
        driveWithEncoders(true);
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        leftMotor.setTargetPosition(left);
        rightMotor.setTargetPosition(right);
        while (opModeIsActive() && (leftMotor.isBusy() || rightMotor.isBusy())) {
            telemetry.addData("encoder drive", "driving");
            telemetry.update();
        }
        telemetry.addData("encoder drive", "complete");
        telemetry.update();
    }

    private void driveWithEncoders(boolean enabled) {
        for (DcMotor motor : new DcMotor[]{leftMotor, rightMotor}) {
            if (enabled) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    public void setClampPower(double power) {
        clampServo.setPower(power);
    }

    private boolean getTouchOpen() {
        return !armTouchOpen.getState();
    }

    private boolean getTouchClosed() {
        return !armTouchClosed.getState();
    }

    public void openClamp() {
        telemetry.addData("clamp", "opening");
        telemetry.update();
        clampServo.setPower(-1);
        while (opModeIsActive() && !getTouchOpen()) ;
        clampServo.setPower(0);
        telemetry.addData("clamp", "opened");
        telemetry.update();
    }

    public void gyroCalibrate() {
        telemetry.addData("gyro", "calibrating");
        telemetry.update();
        gyro.calibrate();
        while (opModeIsActive() && gyro.isCalibrating()) ;
        telemetry.addData("gyro", "calibrated");
        telemetry.update();
    }

    public int gyroHeading() {
        int heading = gyro.getHeading();
        return angleNormalize(heading);
    }

    private int angleNormalize(int angle) {
        angle %= 360;
        if (angle < -180) {
            return angle + 360;
        } else if (angle >= 180) {
            return angle - 360;
        } else {
            return angle;
        }
    }

    //returns -2 for unknown, -1 for left, 0 for centre, 1 for right
    private int targetColumn() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(vuforiaTrackable);
        telemetry.addData("vuforia", vuMark.toString());
        telemetry.update();

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
        int red = jewelColour.red();
        int blue = jewelColour.blue();

        if (Math.abs(red - blue) < JEWEL_COLOUR_THRESHOLD) {
            return 0;
        } else if (red > blue) {
            return 1;
        } else {
            return -1;
        }
    }

    private void setJewelArmPosition(double position) {
        jewelServo.setPosition(position);
    }

    //return -1 for blue, 1 for red
    abstract int teamColour();

    abstract boolean nearRelic();
}