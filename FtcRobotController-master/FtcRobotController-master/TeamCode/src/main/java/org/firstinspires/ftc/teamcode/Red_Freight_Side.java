package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Red Freight Side", group="Linear Opmode")
public class Red_Freight_Side extends BaseAutoOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor drive_FL = null;
    private DcMotor drive_RL = null;
    private DcMotor drive_FR = null;
    private DcMotor drive_RR = null;
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    private DcMotor armExtend = null;
    private DcMotor feeder = null;

    private Servo kickout = null;
    private Servo odometryLift1 = null;
    private CRServo leftDucky = null;
    private CRServo rightDucky = null;

    private DcMotor armEncoder = null;
    private DcMotor extendEncoder = null;
    private DcMotor feedEncoder = null;
    private DcMotor leftEncoder = null;
    private DcMotor rightEncoder = null;
    private DcMotor rearEncoder = null;

    private DistanceSensor LS_distance;
    private DistanceSensor RS_distance;
    private DistanceSensor RL_distance;
    private DistanceSensor RR_distance;
    private DistanceSensor frontDistance;

    BNO055IMU imu;

    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware

    static final double     FORWARD_SPEED = 1;
    static final double     TURN_SPEED    = 1;

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        drive_FL = hardwareMap.get(DcMotor.class, "drive_FL");
        drive_RL = hardwareMap.get(DcMotor.class, "drive_RL");
        drive_FR = hardwareMap.get(DcMotor.class, "drive_FR");
        drive_RR = hardwareMap.get(DcMotor.class, "drive_RR");
        leftArm = hardwareMap.get(DcMotor.class, "left_Arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_Arm");
        armExtend = hardwareMap.get(DcMotor.class, "extend_Arm");
        feeder = hardwareMap.get(DcMotor.class, "feeder");

        kickout = hardwareMap.get(Servo.class, "kickout");
        odometryLift1 = hardwareMap.get(Servo.class,"odometryLift1");
        leftDucky = hardwareMap.get(CRServo.class, "left_Ducky");
        rightDucky = hardwareMap.get(CRServo.class, "right_Ducky");

        LS_distance = hardwareMap.get(DistanceSensor.class, "LS_distance");
        RS_distance = hardwareMap.get(DistanceSensor.class, "RS_distance");
        RL_distance = hardwareMap.get(DistanceSensor.class, "RL_distance");
        RR_distance = hardwareMap.get(DistanceSensor.class, "RR_distance");
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");

        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        drive_FL.setDirection(DcMotor.Direction.FORWARD);
        drive_RL.setDirection(DcMotor.Direction.FORWARD);
        drive_FR.setDirection(DcMotor.Direction.REVERSE);
        drive_RR.setDirection(DcMotor.Direction.REVERSE);

        //Drive Modes
        drive_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Can See Element -", readDisVision1());
        telemetry.update();

        waitForStart();
        runtime.reset();

        //Auto Starts Here
        // 1 Top, 2 Middle, 3 Bottom
        Boolean isMiddle = readDisVision1();

        //Kickout kicks
        kickout.setPosition(0);

        sleep(1500);

        //Moves forward
        forwardsDistanceDrive(5); //12/4/2021 4:36 pm Added missing forward drive -Viassna

        compareBackSensorsNew(); //12/4/2021 4:36 pm Added missing distance compare code -Viassna

        forwardsDistanceDrive(4);

        //Strafes left to check capstone position
        strafeLeftEncoder(.5, 7);

        sleep(500);

        //Moves arm up to according to capstone position
        int angle = calibrateDisVisionAngle(readDisVision2(isMiddle));
        int reduction = calibrateDisVisionReduction(readDisVision2(isMiddle));

        armMoveUp(-angle);

        compareBackSensorsNew(); //12/4/2021 4:46 pm Added missing compare function -Viassna
        //Strafes left to shipping hub
        strafeLeftEncoder(.5, 12);

        sleep(500);

        //Straightens robot
        compareBackSensorsNew();

        //Moves towards shipping hub
        // !NOTE! An idea to fix this is to use the frontDistanceDrive function from the ducky wheel autonomous when its about to spit it out on the wobble -Viassna 12/4/2021 6:08 pm
        forwardsDistanceDrive(23 - reduction);

        sleep(500); //12/4/2021 5:47 Added sleep -Viassna, sleep is needed because feederSpit spits too early

        //Feeder spits starting block
        feederSpit(0.5); //12/4/2021 11:19 am Changed spit speed from .75 to .50 and changed runtime from 3 to 1 second -Viassna

        feeder.setPower(0);

        backwardsDistanceDrive(15);

        //Turns 90 degrees right
        turnRight(90);

        //Moves arm to -11
        armMoveDown(-10); //12/4/2021 - 10:22 am Changed from -5 to -11 -Viassna, 5:05 pm Changed from -11 to -12 -Viassna 5:15 pm Changed from -10 -Viassna

        //Straightens robot
        compareBackSensorsNew();

        //Strafes right until RS_Distance sensor is 2 in away from wall
        strafeRight(1);

        //Extends arm to block pickup arm position
        extendArm(1700); // 12/4/2021 - 9:30 am - Larson Changed from 2200, Changed from 2000 to 1700 -Viassna

        //Moves forward inside warehouse
        runForwardsEncoder(-.6, 58); //12/4/2021 - 9:30 am - Larson Changed from 48, 11:15 am Changed from 50 in to 62 in -Viassna, 5:09 pm Changed from 62 to 60 -Viassna

        //Feeder eats block
        feederEat(-.8); //12/4/2021 11:42 am Added function for feeder to have time to eat block once inside warehouse -Viassna

        feeder.setPower(0);

        //Moves arm up to allow strafing
        armMoveUp(-20); //12/4/2021 5:07 pm Changed from -15 to -20

        //Moves backward 6 in
        //runForwardsEncoder(.6,6);

        //Strafes left 20 in inside warehouse
        strafeLeftEncoder(.5, 20); //12/4/2021 11:11 am Added strafeLeft so robot can be more inside warehouse -Viassna

        //Lifts back odometry wheel
        odometryLift1.setPosition(.5);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Wheel Encoder", drive_FL.getCurrentPosition());
        telemetry.addData("Arm Angle", rightArm.getCurrentPosition() / 20);
        telemetry.addData("Left Arm Power", leftArm.getPower());
        telemetry.addData("Right Arm Power", rightArm.getPower());
        telemetry.addData("Kickout", kickout.getPosition());
        telemetry.addData("Left Ducky Wheel", leftDucky.getPower());
        telemetry.addData("Right Ducky Wheel", rightDucky.getPower());
        telemetry.addData("LS Distnace", String.format("%.01f in", LS_distance.getDistance(DistanceUnit.INCH)));
        telemetry.addData("RS Distance", String.format("%.01f in", RS_distance.getDistance(DistanceUnit.INCH)));
        telemetry.addData("RL Distance", String.format("%.01f in", RL_distance.getDistance(DistanceUnit.INCH)));
        telemetry.addData("RR Distance", String.format("%.01f in", RR_distance.getDistance(DistanceUnit.INCH)));
        telemetry.addData("frontDistance", String.format("%.01f in", frontDistance.getDistance(DistanceUnit.INCH)));
        telemetry.update();
    }

    public void strafeLeftEncoder(double speed, double inputInches) {

        double encoderValue = inchesBore(inputInches);

        double startingValue = drive_RL.getCurrentPosition();

        while (abs(drive_RL.getCurrentPosition()) < abs(startingValue) + abs(encoderValue)) {

            drive_FL.setPower(speed);
            drive_RL.setPower(-speed);
            drive_FR.setPower(-speed);
            drive_RR.setPower(speed);

            telemetry.addData("Encoder Target", encoderValue);
            telemetry.addData("Back Dead Encoder Running", drive_RL.getCurrentPosition());
            telemetry.update();

        }

        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);

        telemetry.addData("Encoder Target", encoderValue);
        telemetry.addData("Back Dead Encoder Finished", drive_RL.getCurrentPosition());
        telemetry.update();

    }

    public boolean readDisVision1() {

        if (frontDistance.getDistance(DistanceUnit.INCH) < 20) {

            return true;

        } else {

            return false;

        }

    }

    public int readDisVision2(boolean Middle) {

        if (Middle == true) {

            return 2;

        } else if (frontDistance.getDistance(DistanceUnit.INCH) < 20) {

            return 3;

        } else {

            return 1;

        }

    }

    public int calibrateDisVisionAngle(int Position) {

        if (Position == 1) {

            return 75;

        } else if (Position == 2){

            return 50;

        } else {

            return 25;

        }

    }

    public int calibrateDisVisionReduction(int Position) {

        if (Position == 1) {

            return 1;

        } else if (Position == 2){

            return 2; //12/4/2021 5:41 pm Changed from 4 to 2 -Viassna

        } else {

            return 3; //12/4/2021 5:41 pm Changed from 6 to 3 -Viassna

        }

    }

    public double inchesBore(double input) {

        final double     COUNTS_PER_MOTOR_REV    = 8192;    // eg: REV Bore Encoder
        final double     DRIVE_GEAR_REDUCTION    = 1;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_INCHES   = 2.83465;     // For figuring circumference
        final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

        return COUNTS_PER_INCH * input;

    }

    public void compareBackSensorsNew() {

        double error = RL_distance.getDistance(DistanceUnit.INCH) - RR_distance.getDistance(DistanceUnit.INCH);

        while (error > .2) {

            error = RL_distance.getDistance(DistanceUnit.INCH) - RR_distance.getDistance(DistanceUnit.INCH);

            drive_FL.setPower(0.2);
            drive_RL.setPower(0.2);
            drive_FR.setPower(-0.2);
            drive_RR.setPower(-0.2);
            telemetry.addData("Error", error);
            telemetry.addData("Left Distance", RL_distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Left Distance", RR_distance.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }

        while (error < -.2) {

            error = RL_distance.getDistance(DistanceUnit.INCH) - RR_distance.getDistance(DistanceUnit.INCH);

            drive_FL.setPower(-0.2);
            drive_RL.setPower(-0.2);
            drive_FR.setPower(0.2);
            drive_RR.setPower(0.2);
            telemetry.addData("Error", error);
            telemetry.addData("Left Distance", RL_distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Left Distance", RR_distance.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }

    }

    public void runForwardsEncoder(double speed, double inputInches) {
        double encoderValue = inchesBore(inputInches);
        double startingValue = drive_RR.getCurrentPosition();

        while (abs(drive_RR.getCurrentPosition()) < abs(startingValue) + abs(encoderValue)) {
            drive_FL.setPower(speed);
            drive_RL.setPower(speed);
            drive_FR.setPower(speed);
            drive_RR.setPower(speed);

            telemetry.addData("Encoder Target", encoderValue);
            telemetry.addData("Right Dead Encoder Running", drive_RR.getCurrentPosition());
            telemetry.update();
        }
        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);

        telemetry.addData("Encoder Target", encoderValue);
        telemetry.addData("Right Dead Encoder Finished", drive_RR.getCurrentPosition());
        telemetry.update();
    }

    public void ResetDriveEncoder() {
        drive_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive_RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive_RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnOnEncoders() {
        drive_FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive_RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive_FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive_RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Kickout() {
        kickout.setPosition(.25);
    }

    public void armMoveUp(int degrees) {

        while (-degreesBore(rightArm.getCurrentPosition()) > degreesBore(degrees) * 20) {

            rightArm.setPower(.3);
            leftArm.setPower(.3);
            telemetry.addData("Current Angle", degreesBore(rightArm.getCurrentPosition()));
            telemetry.addData("Target Degrees", degreesBore(degrees));
            telemetry.update();

        }

        rightArm.setPower(0);
        leftArm.setPower(0);
        telemetry.addData("Current Angle", getAbsoluteAngle());
        telemetry.addData("Target Degrees", degreesBore(degrees));
        telemetry.update();

    }

    public void armMoveDown(int degrees) {

        while (-degreesBore(rightArm.getCurrentPosition()) < degreesBore(degrees) * 20) {

            rightArm.setPower(-.3);
            leftArm.setPower(-.3);
            telemetry.addData("Current Angle", degreesBore(rightArm.getCurrentPosition()));
            telemetry.addData("Target Degrees", degreesBore(degrees));
            telemetry.update();

        }

        rightArm.setPower(0);
        leftArm.setPower(0);
        telemetry.addData("Current Angle", getAbsoluteAngle());
        telemetry.addData("Target Degrees", degreesBore(degrees));
        telemetry.update();

    }

    public void extendArm(int ticks) {

        while (armExtend.getCurrentPosition() < ticks) {

            armExtend.setPower(.3);

        }

        armExtend.setPower(0);

    }

    public int degreesBore(int input) {

        final int COUNTS_PER_BORE_MOTOR_REV = 8192;    // eg: GOBUILDA Motor Encoder
        int COUNTS_TICKS_PER_REV_PER_DEGREE = (COUNTS_PER_BORE_MOTOR_REV) / 360;

        return COUNTS_TICKS_PER_REV_PER_DEGREE * input;
    }

    public void compareBackSensorsStart() {

        double error = RL_distance.getDistance(DistanceUnit.INCH) - RR_distance.getDistance(DistanceUnit.INCH);

        while (error > 1) {

            drive_FL.setPower(0.0);
            drive_RL.setPower(0.0);
            drive_FR.setPower(-0.1);
            drive_RR.setPower(-0.1);

        }

        while (error < -1) {

            drive_FL.setPower(0.0);
            drive_RL.setPower(0.0);
            drive_FR.setPower(-0.1);
            drive_RR.setPower(-0.1);

        }

    }

    public void strafeLeft(double inches) {
        while (LS_distance.getDistance(DistanceUnit.INCH) > inches + 4) {
            drive_FL.setPower(0.6);
            drive_RL.setPower(-0.6);
            drive_FR.setPower(-0.6);
            drive_RR.setPower(0.6);
        }
            drive_FL.setPower(0);
            drive_RL.setPower(0);
            drive_FR.setPower(0);
            drive_RR.setPower(0);
    }

    public void strafeRight(double inches) {
        while (RS_distance.getDistance(DistanceUnit.INCH) > inches + 4) {
            drive_FL.setPower(-0.6);
            drive_RL.setPower(0.6);
            drive_FR.setPower(0.6);
            drive_RR.setPower(-0.6);
        }
        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);
    }

    public void spinDuckyLeft(double speed) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 4.0))
        leftDucky.setPower(speed);
        telemetry.addData("Left Ducky Wheel", runtime.seconds());
        telemetry.update();
    }

    public void feederSpit(double speed) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 1)) //12/4/2021 11:19 am Changed runtime from 3 to 1 seconds -Viassna
            feeder.setPower(speed);
        telemetry.addData("Feeder", runtime.seconds());
        telemetry.update();
    }

    public void feederEat(double speed) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= .750)) //12/4/2021 11:42 am Added function for feeder to have time to eat block -Viassna
            feeder.setPower(speed);
        telemetry.addData("Feeder", runtime.seconds());
        telemetry.update();
    }

    public void forwardsDistanceDrive(int inches) {
        while (RL_distance.getDistance(DistanceUnit.INCH) < inches) {
            drive_FL.setPower(-.3);
            drive_RL.setPower(-.3);
            drive_FR.setPower(-.3);
            drive_RR.setPower(-.3);
        }
            drive_FL.setPower(0);
            drive_RL.setPower(0);
            drive_FR.setPower(0);
            drive_RR.setPower(0);
    }

    public void forwardsDistanceDriveFront(int inches) {
        while (frontDistance.getDistance(DistanceUnit.INCH) > inches) {
            drive_FL.setPower(-.3);
            drive_RL.setPower(-.3);
            drive_FR.setPower(-.3);
            drive_RR.setPower(-.3);
        }
        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);
    }

    public void backwardsDistanceDrive(int inches) {
        while (RL_distance.getDistance(DistanceUnit.INCH) > inches) {
            drive_FL.setPower(.3);
            drive_RL.setPower(.3);
            drive_FR.setPower(.3);
            drive_RR.setPower(.3);
        }
        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);
    }

    public void forwardsDistanceHub(int inches) {
        if (frontDistance.getDistance(DistanceUnit.INCH) > inches) {
            drive_FL.setPower(-.3);
            drive_RL.setPower(-.3);
            drive_FR.setPower(-.3);
            drive_RR.setPower(-.3);
            telemetry.addData("frontDistance", String.format("%.01f in", frontDistance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        } else {
            drive_FL.setPower(0);
            drive_RL.setPower(0);
            drive_FR.setPower(0);
            drive_RR.setPower(0);
            telemetry.addData("frontDistance", String.format("%.01f in", frontDistance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }
    }

    // resets currAngle Value
    public void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {

        // Get current orientation
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public void turnRight(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - abs(getAngle());
            telemetry.addData("error", error);
            telemetry.update();
        }

        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);
    }

    public void turnLeft(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.setMotorPower(motorPower, -motorPower, motorPower, -motorPower);
            error = degrees - abs(getAngle());
            telemetry.addData("error", error);
            telemetry.update();
        }

        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);
    }

    public void turnTo(double degrees){

        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        System.out.println(orientation.firstAngle);
        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        turnRight(error);
    }

    public double getAbsoluteAngle() {
        return robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    void turnToPID(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (Math.abs(targetAngle - getAbsoluteAngle()) > 0.5 || pid.getLastSlope() > 0.75) {
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);

            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        robot.setAllPower(0);
    }
}