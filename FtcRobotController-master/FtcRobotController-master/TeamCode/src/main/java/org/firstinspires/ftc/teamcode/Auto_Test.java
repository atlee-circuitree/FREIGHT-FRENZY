package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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

@Autonomous(name="Auto Test", group="Linear Opmode")
public class Auto_Test extends BaseAutoOpMode {

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

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive_FL = hardwareMap.get(DcMotor.class, "drive_FL");
        drive_RL = hardwareMap.get(DcMotor.class, "drive_RL");
        drive_FR = hardwareMap.get(DcMotor.class, "drive_FR");
        drive_RR = hardwareMap.get(DcMotor.class, "drive_RR");
        leftArm = hardwareMap.get(DcMotor.class, "left_Arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_Arm");
        armExtend = hardwareMap.get(DcMotor.class, "extend_Arm");
        feeder = hardwareMap.get(DcMotor.class, "feeder");

        kickout = hardwareMap.get(Servo.class, "kickout");
        leftDucky = hardwareMap.get(CRServo.class, "left_Ducky");
        rightDucky = hardwareMap.get(CRServo.class, "right_Ducky");

        LS_distance = hardwareMap.get(DistanceSensor.class, "LS_distance");
        RS_distance = hardwareMap.get(DistanceSensor.class, "RS_distance");
        RL_distance = hardwareMap.get(DistanceSensor.class, "RL_distance");
        RR_distance = hardwareMap.get(DistanceSensor.class, "RR_distance");

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

        waitForStart();
        runtime.reset();

        //Auto Starts Here
        runForwardsEncoder(.5,4);

        Kickout();

        //armTurn();

        compareBackSensorsStart();

        strafeLeft();

        spinDuckyLeft(1,3);

        forwardsDistanceDrive1();

        //Add turn 90 degrees right using PID here

        //Add forwardsDistanceDrive2 here

        //Add backwardsDistanceDrive here

        //Add driveRight to alliance storage unit here

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
        telemetry.update();


    }

    public double inchesBore(double input) {

        final double     COUNTS_PER_MOTOR_REV    = 8192;    // eg: REV Bore Encoder
        final double     DRIVE_GEAR_REDUCTION    = 1;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_INCHES   = 2.83465;     // For figuring circumference
        final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

        return COUNTS_PER_INCH * input;

    }

    public void runForwardsEncoder(double speed, double inputInches) {

        double encoderValue = inchesBore(inputInches);

        while (abs(drive_RR.getCurrentPosition()) < encoderValue) {

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

        ResetDriveEncoder();
        turnOnEncoders();


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

    public void armTurn() {
        if (rightArm.getCurrentPosition() < degreesBore(-50)) {

            rightArm.setPower(-.5);
            leftArm.setPower(-.5);
            ResetDriveEncoder();
            turnOnEncoders();
        }
    }

    public int degreesBore(int input) {

        final int COUNTS_PER_BORE_MOTOR_REV = 8192;    // eg: GOBUILDA Motor Encoder
        int COUNTS_TICKS_PER_REV_PER_DEGREE = (COUNTS_PER_BORE_MOTOR_REV) / 360;

        return COUNTS_TICKS_PER_REV_PER_DEGREE * input;
    }

    public void compareBackSensorsStart() {
        if (RL_distance.getDistance(DistanceUnit.INCH) < (RR_distance.getDistance(DistanceUnit.INCH))) {
            drive_FL.setPower(0.5);
            drive_RL.setPower(0.5);
            drive_FR.setPower(-0.5);
            drive_RR.setPower(-0.5);
        } else if (RR_distance.getDistance(DistanceUnit.INCH) < (RL_distance.getDistance(DistanceUnit.INCH))) {
            drive_FL.setPower(-0.5);
            drive_RL.setPower(-0.5);
            drive_FR.setPower(0.5);
            drive_RR.setPower(0.5);
        } else if (RL_distance.getDistance(DistanceUnit.INCH) == (RR_distance.getDistance(DistanceUnit.INCH))) {
            drive_FL.setPower(0);
            drive_RL.setPower(0);
            drive_FR.setPower(0);
            drive_RR.setPower(0);
        }
    }

    public void strafeLeft() {
        if (LS_distance.getDistance(DistanceUnit.INCH) < 30) {
            drive_FL.setPower(-0.5);
            drive_RL.setPower(0.5);
            drive_FR.setPower(0.5);
            drive_RR.setPower(-0.5);
        } else if (LS_distance.getDistance(DistanceUnit.INCH) == 7.75) {
            drive_FL.setPower(0);
            drive_RL.setPower(0);
            drive_FR.setPower(0);
            drive_RR.setPower(0);
        }
    }

    public void spinDuckyLeft(double speed, double seconds) {

        leftDucky.setPower(speed);
    }

    public void forwardsDistanceDrive1() {
        if (RL_distance.getDistance(DistanceUnit.INCH) < 40) {
            drive_FL.setPower(1);
            drive_RL.setPower(1);
            drive_FR.setPower(1);
            drive_RR.setPower(1);
        } else {
            drive_FL.setPower(0);
            drive_RL.setPower(0);
            drive_FR.setPower(0);
            drive_RR.setPower(0);
        }
    }
}