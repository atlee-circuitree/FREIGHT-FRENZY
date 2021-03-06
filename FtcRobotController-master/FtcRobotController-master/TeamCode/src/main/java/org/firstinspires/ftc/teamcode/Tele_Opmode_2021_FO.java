package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.kauailabs.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.kauailabs.navx.ftc.navXPIDController;


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

@TeleOp(name="TeleOp Field Orientation Test", group="Linear Opmode")
@Disabled
public class Tele_Opmode_2021_FO extends BaseAutoOpMode {

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
    private Servo claw = null;
    private Servo odometryLift1 = null;

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
        claw = hardwareMap.get(Servo.class, "claw");
        odometryLift1 = hardwareMap.get(Servo.class, "odometryLift1");

        leftArm.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotor.Direction.FORWARD);

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Slowmode Variables
        double SV = 1;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the batter;
        drive_FL.setDirection(DcMotor.Direction.FORWARD);
        drive_RL.setDirection(DcMotor.Direction.FORWARD);
        drive_FR.setDirection(DcMotor.Direction.REVERSE);
        drive_RR.setDirection(DcMotor.Direction.REVERSE);

        //Drive Modes
        drive_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GetHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        claw.setPosition(1);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Show encoder values on the phone
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Wheel Encoder", drive_FL.getCurrentPosition());
            telemetry.addData("Arm Encoder", rightArm.getCurrentPosition());
            telemetry.addData("Arm Angle", rightArm.getCurrentPosition() / 20);
            telemetry.addData("Left Arm Power", leftArm.getPower());
            telemetry.addData("Right Arm Power", rightArm.getPower());
            telemetry.addData("Kickout", kickout.getPosition());
            telemetry.addData("Left Ducky Wheel", leftDucky.getPower());
            telemetry.addData("Right Ducky Wheel", rightDucky.getPower());
            telemetry.addData("Feeder", feeder.getCurrentPosition());
            telemetry.addData("Arm Extend", armExtend.getCurrentPosition());
            telemetry.addData("Arm Extend Power", armExtend.getPower());
            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.update();

            //Mecanum Drive Code

            double y_stick = gamepad1.left_stick_y;
            double x_stick = gamepad1.left_stick_x;

            double pi = 3.1415926;

            /* Adjust Joystick X/Y inputs by navX MXP yaw angle */

            double gyro_degrees = navx_centered.getYaw();
            double gyro_radians = gyro_degrees * pi/180;
            double temp = y_stick * cos(gyro_radians) + x_stick * sin(gyro_radians);
            x_stick = -y_stick * sin(gyro_radians) + x_stick * cos(gyro_radians);

            /* At this point, Joystick X/Y (strafe/forwrd) vectors have been */
            /* rotated by the gyro angle, and can be sent to drive system */

            double r = Math.hypot(x_stick, y_stick);
            double robotAngle = Math.atan2(y_stick, -x_stick) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = (r * cos(robotAngle) + rightX);
            final double v2 = (r * Math.sin(robotAngle) - rightX);
            final double v3 = (r * Math.sin(robotAngle) + rightX);
            final double v4 = (r * cos(robotAngle) - rightX);

            drive_FL.setPower(v1 * .7);
            drive_RL.setPower(v3 * .7);
            drive_FR.setPower(v2 * .7);
            drive_RR.setPower(v4 * .7);

            //Controller 1
            //Controls Kickout
            if (gamepad1.b) {
                kickout.setPosition(0);
            }

            if (gamepad1.a) {
                kickout.setPosition(1);
            }

            //Spins ducky wheels
            //LeftDucky = Spin only for Red Ducky Carousel
            //Right = Spin only for Blue Ducky Carousel
            if (gamepad1.left_bumper) {
                leftDucky.setPower(-1);
                rightDucky.setPower(-1);
            } else if (gamepad1.right_bumper) {
                leftDucky.setPower(1);
                rightDucky.setPower(1);
            } else {
                leftDucky.setPower(0);
                rightDucky.setPower(0);
            }

            if (gamepad1.y)
                odometryLift1.setPosition(.5);


            //Controller 2
            //Slows arm
            if (gamepad2.a) {
                SV = .5;
            } else {
                SV = 1;
            }

            //Manually turns arm :)
            if (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1) {
                leftArm.setPower(gamepad2.left_stick_y * .75);
                rightArm.setPower(gamepad2.left_stick_y * .75);
            } else  {
                leftArm.setPower(0);
                rightArm.setPower(0);
            }

            //Turns Feeder Motor Inward
            if (gamepad2.left_trigger > .5) {
                feeder.setPower(.5);
            } else if (gamepad2.right_trigger > .5) {
                feeder.setPower(-1);
            } else {
                feeder.setPower(0);
            }


            //Extends and Retracts the arm
            if (gamepad2.x) {
                armExtend.setPower(.3);
            } else if (gamepad2.y) {
                armExtend.setPower(-.3);
            } else {
                armExtend.setPower(0);
            }


            //Moves claw n stuff
            if (gamepad2.left_bumper) {
                claw.setPosition(.3);
            }

            if (gamepad2.right_bumper) {
                claw.setPosition(0);
            }
        }
    }

    public int degreesBore(int input) {

        final int COUNTS_PER_BORE_MOTOR_REV = 8192;    // eg: GOBUILDA Motor Encoder
        int COUNTS_TICKS_PER_REV_PER_DEGREE = (COUNTS_PER_BORE_MOTOR_REV) / 360 * 2;

        return COUNTS_TICKS_PER_REV_PER_DEGREE * input;

    }
}