package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public abstract class Autonomous extends LinearOpMode {
    private static final double TICKS_PER_MOTOR_REVOLUTION = 1120;

    private static final double WHEEL_CIRCUMFERENCE_IN = 90 * 0.0393701 * Math.PI;
    private static final double DRIVE_GEAR_RATIO = 45.0 / 30;
    private static final double TURN_DISTANCE = 12.75 * Math.PI;

    private static final double GLYPH_OFFSET_RIGHT = 5.75; //2.8515;

    static final double DRIVE_POWER_FREE = 1.0 / 4; //1.0 / 3;
    static final double DRIVE_POWER = 7.0 / 32; //1.0 / 8; //4.0 / 16;
    static final double TURN_POWER = 3.0 / 32; //1.0 / 8; //3.0 / 16; //0.3; //1.0 / 2;
    static final double TURN_POWER_JEWEL = 2.0 / 32;

    private static final double ENCODER_TURN_MULTIPLIER_BOARD = 1.021;
    private static final double ENCODER_TURN_MULTIPLIER_FOAM = 1.062; //1; //1.054; //experimentally determined

    private CRServo clampServo;

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private DigitalChannel armTouchOpen;

    private Servo jewelServo;
    private ColorSensor jewelColour;

    private VuforiaTrackable vuforiaTrackable;

    private ModernRoboticsI2cGyro gyro;

    private static final int GYRO_TURN_THRESHOLD = 2;

    private int useGyro = 0; //0 for none, 1 for gyro, 2 for imu
    private boolean useVuforia = true;

    private int targetHeading = 0;

    private ElapsedTime autonomousTimer;

    private BNO055IMU imu;

    private final int TIME_LIMIT = 28000;

    @Override
    public void runOpMode() {
        initHardware();

        waitForStart();

        autonomousTimer = new ElapsedTime();

        autonomous();
    }

    void sensorTelemetry(String message) {
        telemetry.addData(message, autonomousTimer);

        telemetry.addData("position", "team %s near relic %s", teamColour(), nearRelic());

        int raw = jewelColourRaw();
        telemetry.addData("jewel", "colour %d (raw %d)", jewelColour(raw), raw);

        telemetry.addData("motor", "left %.4f right %.4f", leftMotor.getPower(), rightMotor.getPower());

        telemetry.addData("column", targetColumn());

        telemetry.addData("heading", gyroHeading());

        telemetry.update();
    }

    void sensorTelemetryLoop() {
        while (opModeIsActive()) {
            sensorTelemetry("sensor telemetry loop");
        }
    }

    void autonomous() {
        jewel();

        if (useVuforia) {
            int column = getColumn();
            glyph(column);
        } else {
            //glyph in centre column
            glyph(0);
        }

        sensorTelemetry("autonomous complete");
    }

    private void jewel() {
        sensorTelemetry("jewel");

        //drives slightly closer to jewel
        double adjustment = -1.0;

        //amount to turn to reread jewel colour
        int turnRead = 4;

        //degrees to turn to knock off jewel
        //positive is clockwise
        int turnDegrees = 25;

        //number of times to turn and retry reading jewel
        int maxTries = 3;

        //lower jewel arm
        setJewelArmPosition(0);
        sleep(1000);

        //drive closer
        encoderDrive(adjustment, DRIVE_POWER);

        //get jewel colour
        sensorTelemetry("jewel read colour");
        int jewelColour = jewelColour(jewelColourRaw());
        int tries = 0;
        int turned = 0;

        //if uncertain, get closer and read again, repeat
        while (jewelColour == 0 && tries < maxTries && opModeIsActive()) {
            tries++;

            turn(turnRead, TURN_POWER, true);
            turned += turnRead;

            sensorTelemetry("jewel reread, try " + tries);
            jewelColour = jewelColour(jewelColourRaw());
        }

        sensorTelemetry("jewel read complete, turning back");
        turn(-turned, TURN_POWER, true);

        //if certain about jewel colour
        if (jewelColour != 0) {
            sensorTelemetry("jewel identified " + jewelColour);

            if (jewelColour == teamColour()) {
                //our colour sensor faces right
                //facing our own jewel, turn ccw
                turnDegrees *= -1;
            }

            //turns to knock off the jewel
            turn(turnDegrees, TURN_POWER_JEWEL, true);

            //raise jewel arm
            setJewelArmPosition(1);

            //turns back
            turn(-turnDegrees, TURN_POWER_JEWEL, true);
        } else {
            sensorTelemetry("jewel not identified");

            //raise jewel arm, abandon attempt
            setJewelArmPosition(1);
        }

        //backs away to starting position
        encoderDrive(-adjustment, DRIVE_POWER);
    }

    void sleep() {
        sleep(50);
    }

    private int getColumn() {
        sensorTelemetry("identifying column");

        int turn = -20;
        int turnMore = -3;
        int turned = turn;

        int tries = 0;
        int maxTries = 3;

        //turns toward vision target
        turn(turn, TURN_POWER, true);

        //gets column value
        int column = targetColumn();

        while ((column > 1 || column < -1) && tries < maxTries) {
            tries++;
            sensorTelemetry("column reread, try " + tries);

            turned += turnMore;

            encoderTurn(turnMore, TURN_POWER, true);
            column = targetColumn();
        }

        sensorTelemetry("column " + column);

        if (column < -1 || column > 1) {
            column = 0;
        }

        //turns back
        turn(-turned, TURN_POWER, true);

        return column;
    }

    private void glyph(int column) {
        sensorTelemetry("scoring glyph");

        //we drive a little farther because of dismounting the balancing stone
        double balanceDismountOffset = -1;

        //time spent driving into cryptobox
        int cryptoboxDriveMillis = 3000;

        //distance to back away at the end
        double backAway = -4;

        //here we multiply by teamColour(), an integer representing -1 for blue or 1 for red
        //this reverses our turning direction if we are on the opposite alliance while
        //preventing unnecessary code duplication

        //if we are on the balancing stone closer to the relic scoring zone
        if (nearRelic()) {
            //turns to face cryptobox
            turn(90 * teamColour(), TURN_POWER, true);

            //drives off and aligns with correct column
            encoderDrive(36 + balanceDismountOffset + glyphHorizontalOffset(column), DRIVE_POWER);
        } else {
            //turns to dismount balancing stone
            turn(90 * teamColour(), TURN_POWER, true);

            //dismounts balancing stone
            encoderDrive(24 + balanceDismountOffset, DRIVE_POWER);

            //turns parallel to cryptobox
            turn(-(90) * teamColour(), TURN_POWER, false);

            //aligns with correct column
            encoderDrive(glyphHorizontalOffset(column) + 12, DRIVE_POWER);
        }

        //turns to face correct column
        turn(90 * teamColour(), TURN_POWER, false);

        double stopTime;

        openClamp();
        sleep(1000);

        stopTime = Math.min(TIME_LIMIT, autonomousTimer.milliseconds() + cryptoboxDriveMillis);
        freeTimeout(stopTime, DRIVE_POWER_FREE, DRIVE_POWER_FREE);

        stopClamp();

        //backs away from glyph so we aren't touching it
        encoderDrive(backAway, DRIVE_POWER);
    }

    private void freeTimeout(double stopTime, double left, double right)
    {
        freeDrive(DRIVE_POWER_FREE, DRIVE_POWER_FREE);
        while (autonomousTimer.milliseconds() < stopTime && opModeIsActive()) {
            if (getTouchOpen() && Math.abs(clampServo.getPower()) > 0.1)
            {
                stopClamp();
            }
            sensorTelemetry("free drive, stop " + stopTime);
        }
        stopDrive();
        sleep();
    }

    private double glyphHorizontalOffset(int column) {
        double columnOffset = 7.5;

        //compensates for the offset of our target column
        //column is an int, -1 is left, 0 is centre, 1 is right
        //teamColour is our team colour, -1 for blue, 1 for red
        //red means left is farther (approach from right)
        //blue means right is farther (approach from left)

        double offset = column * columnOffset * teamColour() * -1;

        //compensates for our glyph mechanism design
        //the glyph is closer to the right side
        //if approaching from right (red), need to drive farther
        //otherwise, need to drive less

        offset += GLYPH_OFFSET_RIGHT * teamColour();

        return offset;
    }

    void initHardware() {
        telemetry.addData("hardware initialization", "working");
        telemetry.update();

        clampServo = hardwareMap.crservo.get("clampServo");

        armTouchOpen = hardwareMap.get(DigitalChannel.class, "tsOpen");
        armTouchOpen.setMode(DigitalChannel.Mode.INPUT);

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        jewelServo = hardwareMap.servo.get("jewelServo");
        jewelColour = hardwareMap.colorSensor.get("jewelColor");

        initVuforia();

        switch (useGyro) {
            case 1:
                gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
                gyro.calibrate();
                ElapsedTime gyroCalibrate = new ElapsedTime();
                while (opModeIsActive() && gyro.isCalibrating()) {
                    telemetry.addData("gyro calibrating", gyroCalibrate);
                    telemetry.update();
                }
                break;

            case 2:
                BNO055IMU.Parameters parameters_imu = new BNO055IMU.Parameters();
                parameters_imu.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parameters_imu.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters_imu.loggingEnabled = false;
                parameters_imu.accelerationIntegrationAlgorithm = new
                        JustLoggingAccelerationIntegrator();
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                telemetry.addLine("imu calibrating");
                imu.initialize(parameters_imu);
                break;

            default:
                break;
        }

        telemetry.addData("hardware initialization", "complete");
        telemetry.update();
    }

    void turn(int angle, double power, boolean board) {
        targetHeading = angleNormalize(targetHeading + angle);
        if (useGyro > 0) {
            gyroTurn(targetHeading, TURN_POWER);
        } else {
            encoderTurn(angle, power, board);
        }
    }

    private void encoderTurn(int angle, double power, boolean board) {
        sensorTelemetry("encoder turn " + angle + " degrees");

        double robotRotations = angle / 360.0;
        double distance = robotRotations * TURN_DISTANCE * (board ? ENCODER_TURN_MULTIPLIER_BOARD : ENCODER_TURN_MULTIPLIER_FOAM);


        encoderDrive(distance, -distance, power);
    }

    private void gyroTurn(int targetHeading, double power) {
        int error = angleNormalize(targetHeading - gyroHeading());

        double leftPower = (error > 0 ? -1 : 1) * power;
        double rightPower = -leftPower;
        freeDrive(leftPower, rightPower);

        while (Math.abs(error) >= GYRO_TURN_THRESHOLD
                && opModeIsActive()) {
            error = angleNormalize(
                    targetHeading - gyroHeading());
            sensorTelemetry("turn to heading "
                    + targetHeading + " error " + error);
        }

        stopDrive();
        sleep();
    }

    private int angleNormalize(int degrees) {
        while (degrees < -180) {
            degrees += 360;
        }

        while (degrees >= 180) {
            degrees -= 360;
        }

        return degrees;
    }

    void freeDrive(double left, double right) {
        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }

    void stopDrive() {
        freeDrive(0, 0);
    }

    private int gyroHeading() {
        switch (useGyro) {
            case 1:
                return -gyro.getHeading();
            case 2:
                return Math.round(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            default:
                return 0;
        }
    }

    void encoderDrive(double inches, double power) {
        encoderDrive(inches, inches, power);
    }

    private void encoderDrive(double leftInches, double rightInches, double power) {
        int left = inchesToEncoderTicks(leftInches);
        int right = inchesToEncoderTicks(rightInches);

        encoderDriveTicks(left, right, power);
    }

    private int inchesToEncoderTicks(double inches) {
        double wheelRotations = inches / WHEEL_CIRCUMFERENCE_IN;
        double shaftRotations = wheelRotations / DRIVE_GEAR_RATIO;
        return (int) Math.round(shaftRotations * TICKS_PER_MOTOR_REVOLUTION);
    }

    private void encoderDriveTicks(int left, int right, double power) {
        if (left == 0 && right == 0) {
            return;
        }

        left += leftMotor.getCurrentPosition();
        right += rightMotor.getCurrentPosition();

        leftMotor.setTargetPosition(left);
        rightMotor.setTargetPosition(right);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(power);
        rightMotor.setPower(power);

        while (opModeIsActive() && encodersBusy()) {
            //sensorTelemetry("encoder drive");
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep();
    }

    private boolean encodersBusy() {
        return leftMotor.isBusy() || rightMotor.isBusy();
    }

    private boolean getTouchOpen() {
        return !armTouchOpen.getState();
    }

    private void openClamp()
    {
        clampServo.setPower(-1);
    }

    private void stopClamp()
    {
        clampServo.setPower(0);
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

    private int jewelColourRaw() {
        //get relevant colour sensor values
        int red = jewelColour.red();
        int blue = jewelColour.blue();

        //how much more red there is than blue
        int colourRaw = red - blue;

        return colourRaw;
    }

    private int jewelColour(int colourRaw) {
        //value of red - blue below which we consider this a blue jewel
        int blueThreshold = -48;
        //value of red - blue above which we consider this a red jewel
        int redThreshold = 64;

        if (colourRaw < blueThreshold) {
            //below blue threshold, return -1 for blue
            return -1;
        } else if (colourRaw > redThreshold) {
            //above red threshold, return 1 for red
            return 1;
        } else {
            //unsure about jewel colour
            return 0;
        }
    }

    private void setJewelArmPosition(double position) {
        jewelServo.setPosition(position);
    }

    //return -1 for blue, 1 for red
    abstract int teamColour();

    //whether near relic recovery zone
    abstract boolean nearRelic();
}