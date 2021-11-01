package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@Autonomous(name="Long - Auto Opmode Encoders", group="Linear Opmode")
public class Long_Auto_Opmode_Encoders extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive1 = null;
    private DcMotor rightDrive1 = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive1  = hardwareMap.get(DcMotor.class, "drive_FL");
        rightDrive1 = hardwareMap.get(DcMotor.class, "drive_FR");
        leftDrive2  = hardwareMap.get(DcMotor.class, "drive_RL");
        rightDrive2 = hardwareMap.get(DcMotor.class, "drive_RR");

        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);

        leftDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turnOnEncoders();


        waitForStart();
        runtime.reset();

            runForwardsEncoder(.5, 20);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("FL Encoder Value",leftDrive1.getCurrentPosition());
            telemetry.addData("FR Encoder Value",rightDrive1.getCurrentPosition());
            telemetry.addData("RL Encoder Value",leftDrive2.getCurrentPosition());
            telemetry.addData("RR Encoder Value",rightDrive2.getCurrentPosition());
            telemetry.update();


    }

    public double inches(double input) {

        final double     COUNTS_PER_MOTOR_REV    = 383.6;    // eg: GOBUILDA Motor Encoder
        final double     DRIVE_GEAR_REDUCTION    = 1;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_INCHES   = 3.93701;     // For figuring circumference
        final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

        return COUNTS_PER_INCH * input;

    }

    public void runForwardsEncoder(double speed, double inputInches) {

        double encoderValue = inches(inputInches);

        while (leftDrive2.getCurrentPosition() < encoderValue) {

            leftDrive1.setPower(speed);
            leftDrive2.setPower(speed);
            rightDrive1.setPower(speed);
            rightDrive2.setPower(speed);

        }

        leftDrive1.setPower(0);
        leftDrive2.setPower(0);
        rightDrive1.setPower(0);
        rightDrive2.setPower(0);

        ResetDriveEncoder();

    }

    public void ResetDriveEncoder(){

        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void turnOnEncoders() {

        leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}

