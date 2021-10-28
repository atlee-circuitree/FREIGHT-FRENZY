package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name="Mecanum Opmode", group="Linear Opmode")
public class Mecanum_Opmode_2021 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive1 = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive1 = null;
    private DcMotor rightDrive2 = null;
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    private DcMotor armExtend = null;
    private DcMotor feeder = null;
    private Servo kickout = null;
    private Servo leftDucky = null;
    private Servo rightDucky = null;
    private Sensor NAVX = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive1 = hardwareMap.get(DcMotor.class, "drive_FL");
        leftDrive2 = hardwareMap.get(DcMotor.class, "drive_RL");
        rightDrive1 = hardwareMap.get(DcMotor.class, "drive_FR");
        rightDrive2 = hardwareMap.get(DcMotor.class, "drive_RR");
        leftArm = hardwareMap.get(DcMotor.class, "left_Arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_Arm");
        armExtend = hardwareMap.get(DcMotor.class, "extend_Arm");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        kickout = hardwareMap.get(Servo.class, "kickout");
        leftDucky = hardwareMap.get(Servo.class, "left_Ducky");
        rightDucky = hardwareMap.get(Servo.class, "right_Ducky");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.left_stick_y;
            double turn  = -gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            leftDrive1.setPower(v1);
            leftDrive2.setPower(v3);
            rightDrive1.setPower(v2);
            rightDrive2.setPower(v4);
        }
    }
}