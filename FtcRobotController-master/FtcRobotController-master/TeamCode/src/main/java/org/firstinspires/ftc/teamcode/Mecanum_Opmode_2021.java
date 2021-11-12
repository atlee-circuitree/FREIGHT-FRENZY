package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.util.stream.LongStream.range;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.stream.IntStream;


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

@TeleOp(name="Mecanum Opmode", group="Linear Opmode")
public class Mecanum_Opmode_2021 extends LinearOpMode {

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

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
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

        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        drive_FL.setDirection(DcMotor.Direction.FORWARD);
        drive_RL.setDirection(DcMotor.Direction.FORWARD);
        drive_FR.setDirection(DcMotor.Direction.REVERSE);
        drive_RR.setDirection(DcMotor.Direction.FORWARD);

        //Drive Modes
        drive_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Show encoder values on the phone
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Left Arm", leftArm.getCurrentPosition());
            telemetry.addData("Right Arm", rightArm.getCurrentPosition());
            telemetry.addData("Left Arm Power", leftArm.getPower());
            telemetry.addData("Right Arm Power", rightArm.getPower());
            telemetry.addData("Kickout", kickout.getPosition());
            telemetry.addData("Left Ducky Wheel", leftDucky.getPower());
            telemetry.addData("Right Ducky Wheel", rightDucky.getPower());
            telemetry.addData("Feeder", feeder.getCurrentPosition());
            telemetry.addData("Arm Extend", armExtend.getCurrentPosition());
            telemetry.addData("Arm Extend Power", armExtend.getPower());
            telemetry.update();

            //Mecanum Drive Code
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            drive_FL.setPower(v1);
            drive_RL.setPower(v3);
            drive_FR.setPower(v2);
            drive_RR.setPower(v4);

            //Controller 1
            //Controls Kickout
            if (gamepad1.b) {
                kickout.setPosition(7);
            }


            //Controls Left Ducky Wheel
            if (gamepad1.left_bumper) {
                leftDucky.setPower(-.5);
            } else {
                leftDucky.setPower(0);
            }

            //Controls Right Ducky Wheel
            if (gamepad1.right_bumper) {
                rightDucky.setPower(.5);
            } else {
                rightDucky.setPower(0);
            }


            //Turns Feeder Motor Inward
            if (gamepad1.left_trigger > .5) {
                feeder.setPower(1);
            } else {
                feeder.setPower(0);
            }

            //Turns Feeder Motor Outward
            if (gamepad1.right_trigger > .5) {
                feeder.setPower(-1);
            } else {
                feeder.setPower(0);
            }

            //Controller 2
            //Manually turns arm in case it doesn't extend and turn to set position automatically (FAILSAFE)
            if (gamepad2.dpad_up) {
                leftArm.setPower(-1);
                rightArm.setPower(-1);
            } else {
                leftArm.setPower(0);
                rightArm.setPower(0);
            }

            if (gamepad2.dpad_down) {
                leftArm.setPower(1);
                rightArm.setPower(1);
            } else {
                leftArm.setPower(0);
                rightArm.setPower(0);
            }

            //Extends/retracts arm to set position in case it doesn't extend and turn to set position automatically (FAILSAFE)
            if (gamepad2.dpad_left) {
                armExtend.setTargetPosition((-200));
                armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.dpad_right) {
                armExtend.setTargetPosition(0);
                armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            //Moves arm to set levels

            //Pickup level
            if (gamepad2.a) {

                if (rightArm.getCurrentPosition() < degreesBore(0) - 20) {

                    rightArm.setPower((abs(rightArm.getCurrentPosition() - degreesBore(10) / 3500) ) + .2);
                    leftArm.setPower((abs(rightArm.getCurrentPosition() - degreesBore(10) / 3500) ) + .2);

                } else if (rightArm.getCurrentPosition() > degreesBore(0) + 20) {

                    rightArm.setPower(-((abs(rightArm.getCurrentPosition() - degreesBore(10) / 3500) ) + .2));
                    leftArm.setPower(-((abs(rightArm.getCurrentPosition() - degreesBore(10) / 3500) ) + .2));

                } else {

                    rightArm.setPower(0);
                    leftArm.setPower(0);

                }

            }

            //Medium level
            if (gamepad2.a) {

                if (rightArm.getCurrentPosition() < degreesBore(35) - 20) {

                    rightArm.setPower((abs(rightArm.getCurrentPosition() - degreesBore(10) / 3500) ) + .2);
                    leftArm.setPower((abs(rightArm.getCurrentPosition() - degreesBore(10) / 3500) ) + .2);

                } else if (rightArm.getCurrentPosition() > degreesBore(35) + 20) {

                    rightArm.setPower(-((abs(rightArm.getCurrentPosition() - degreesBore(10) / 3500) ) + .2));
                    leftArm.setPower(-((abs(rightArm.getCurrentPosition() - degreesBore(10) / 3500) ) + .2));

                } else {

                    rightArm.setPower(0);
                    leftArm.setPower(0);

                }

            }

            //High Level
            if (gamepad2.y) {

            }

            //Capstone level
            if (gamepad2.x) {

            }

        }

    }

    public int degreesBore(int input) {

        final int COUNTS_PER_BORE_MOTOR_REV = 8192;    // eg: GOBUILDA Motor Encoder
        int COUNTS_TICKS_PER_REV_PER_DEGREE = (COUNTS_PER_BORE_MOTOR_REV) / 360;

        return COUNTS_TICKS_PER_REV_PER_DEGREE * input;

    }
}