package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.max;

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

@Autonomous(name="Blue Freight Cycle", group="Linear Opmode")
public class Blue_Freight_Cycle extends BaseAutoOpMode {

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
    private DistanceSensor frontDistanceLeft;
    private DistanceSensor frontDistanceRight;

    private Servo armTurn;

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
        frontDistanceLeft = hardwareMap.get(DistanceSensor.class, "frontDistanceLeft");
        frontDistanceRight = hardwareMap.get(DistanceSensor.class, "frontDistanceRight");
        armTurn = hardwareMap.get(Servo.class, "armTurn");

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

        if (readDisVision() == 2) {

            telemetry.addLine("Target : Middle");
            telemetry.update();

        } else if (readDisVision() == 3) {

            telemetry.addLine("Target : Low");
            telemetry.update();

        } else {

            telemetry.addLine("Target : High");
            telemetry.update();

        }

        waitForStart();
        runtime.reset();

        //Auto Starts Here

        kickout.setPosition(0);

        sleep(200);

        //Moves feeder box to face straight
        armTurn.setPosition(.44);

        sleep(1000);

        //Moves forward from starting position
        forwardsDistanceDrive(.7, 5);

        //Moves arm up to according to capstone position
        int angle = calibrateDisVisionAngle(readDisVision());
        int reduction = calibrateDisVisionReduction(readDisVision());

        //Strafes right to center line of wobble
        strafeRightEncoder(.8, 13);

        sleep(250);

        //Angle adjust
        compareBackSensorsNew();

        //Moves forwards to Blue Alliance Wobble and raises arm to appropriate level for starting block
        //4-4-2022 5:26 changed from 18 to 15  // 4/4//22 changed 15 to 21
        runForwardsDistanceAndRaiseArm(.6, 19 - reduction, angle);

        //Feeder spits starting block
        feederSpit(0.5);

        feeder.setPower(0);

        //Backs away from Blue Alliance Wobble
        backwardsDistanceDrive(.7, 15);

        //Turns 90 degrees left
        turnLeft(90);

        //Angle adjust
        compareBackSensorsNew();

        //Strafes left until Left Side Distance sensor is .5 in away from wall //did a lil tinkering on the kind of strafe -Nolan 4/5/22
        strafeLeft(.7, 1);

        //Moves forward inside warehouse extending and lowering arm for second block
        runForwardsEncoderAndLowerArmAndExtend(.9, 50, 13);

        feeder.setPower(0);

        //Backs out of warehouse using the left wall from robot to guide it out ( possible issue )
        /*strafeDiagonalDownLeftEncoderAndRaiseArm(1,50,50);

        //Strafes right away from wall
        strafeRightEncoder(.8, 6);

        //Turns right
        turnRight(45);

        //Angle adjust
        compareBackSensorsNew();

        //Moves forwards to wobble and raises arm to second level
        runForwardsDistanceAndRaiseArm(.7, 14, 50);

        //Spits second block out
        feederSpit(0.5);

        //Backs away from Blue Alliance Wobble
        backwardsDistanceDrive(.7, 15);

        //Turns left
        turnLeft(90);

        //Angle adjust
        compareBackSensorsNew();

        //Strafes left until Left Side Distance sensor is .5 in away from wall (LS_distance)
        strafeLeft(.7, .5);

        runForwardsEncoderAndLowerArmAndExtend(.9, 56, 13);*/


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

    public void runForwardsDistanceAndLowerArmAndExtend(double speed, double inches, int angle) {

        feederEat(-.6);

        while (frontDistanceLeft.getDistance(DistanceUnit.INCH) > inches || degreesBore(rightArm.getCurrentPosition()) > degreesBore(angle) * 20 || armExtend.getCurrentPosition() < 1700) {

            if (frontDistanceLeft.getDistance(DistanceUnit.INCH) > inches) {

                drive_FL.setPower(-speed);
                drive_RL.setPower(-speed);
                drive_FR.setPower(-speed);
                drive_RR.setPower(-speed);

            } else {

                drive_FL.setPower(0);
                drive_RL.setPower(0);
                drive_FR.setPower(0);
                drive_RR.setPower(0);

            }

            if (degreesBore(rightArm.getCurrentPosition()) > degreesBore(angle) * 20) {

                rightArm.setPower(-.6);
                leftArm.setPower(-.6);

            } else {

                rightArm.setPower(0);
                leftArm.setPower(0);

            }

            if (armExtend.getCurrentPosition() < 1700) {

                armExtend.setPower(1);

            } else {

                armExtend.setPower(0);

            }

        }

        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);
        rightArm.setPower(0);
        leftArm.setPower(0);
        armExtend.setPower(0);

        sleep(1000);

    }

    public void runForwardsEncoderAndLowerArmAndExtend(double speed, double inches, int angle) {

        armTurn.setPosition(.44);

        feederEat(-.6);

        double currentValue = drive_FL.getCurrentPosition();
        double targetValue = inchesBore(inches) + currentValue;

        while (currentValue < targetValue || degreesBore(rightArm.getCurrentPosition()) > degreesBore(angle) * 20 || armExtend.getCurrentPosition() < 1700) {

            if (currentValue < targetValue) {

               // drive_FL.setPower(-speed);
                //drive_RL.setPower(-speed);
               // drive_FR.setPower(-speed);
                //drive_RR.setPower(-speed);
                driveTrain(-speed, -speed, -speed, -speed, PIDadjustment(1/inchesBore(2), targetValue, currentValue));

                telemetry.addData("Encoder Target", targetValue);
                telemetry.addData("Right Dead Encoder Running", drive_RR.getCurrentPosition());
                telemetry.update();
                currentValue = drive_FL.getCurrentPosition();

            } else {

                drive_FL.setPower(0);
                drive_RL.setPower(0);
                drive_FR.setPower(0);
                drive_RR.setPower(0);

                telemetry.addData("Encoder Target", targetValue);
                telemetry.addData("Right Dead Encoder Finished", drive_RR.getCurrentPosition());
                telemetry.update();

                ResetDriveEncoder();
                turnOnEncoders();

            }

            if (degreesBore(rightArm.getCurrentPosition()) > degreesBore(angle) * 20) {

                rightArm.setPower(-.6);
                leftArm.setPower(-.6);

            } else {

                rightArm.setPower(0);
                leftArm.setPower(0);

            }

            if (armExtend.getCurrentPosition() < 1700) {

                armExtend.setPower(1);

            } else {

                armExtend.setPower(0);

            }

        }

        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);
        rightArm.setPower(0);
        leftArm.setPower(0);
        armExtend.setPower(0);

        sleep(1000);

    }

    public void strafeRightFromWallAndRaiseArm(double speed, double inches, int angle) {

        while (LS_distance.getDistance(DistanceUnit.INCH) < inches + 4 || degreesBore(rightArm.getCurrentPosition()) < degreesBore(angle) * 20) {

            if (LS_distance.getDistance(DistanceUnit.INCH) < inches + 4) {

                drive_FL.setPower(-speed);
                drive_RL.setPower(speed);
                drive_FR.setPower(speed);
                drive_RR.setPower(-speed);

            } else {

                drive_FL.setPower(0);
                drive_RL.setPower(0);
                drive_FR.setPower(0);
                drive_RR.setPower(0);

            }

            if (degreesBore(rightArm.getCurrentPosition()) > degreesBore(angle) * 20) {

                rightArm.setPower(.3);
                leftArm.setPower(.3);

            } else {

                rightArm.setPower(0);
                leftArm.setPower(0);

            }

        }
        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);
        rightArm.setPower(0);
        leftArm.setPower(0);

    }


    public void runForwardsDistanceAndRaiseArm(double speed, double inches, int angle) {
        double currentDistance;

        currentDistance = RL_distance.getDistance(DistanceUnit.INCH);
        while (currentDistance < inches || degreesBore(rightArm.getCurrentPosition()) < degreesBore(angle) * 20) {


            if (currentDistance < inches) {

                //drive_FL.setPower(-speed);
                //drive_RL.setPower(-speed);
                //drive_FR.setPower(-speed);
               //drive_RR.setPower(-speed);
                driveTrain(-speed, -speed, -speed, -speed, PIDadjustment(0.5, inches, currentDistance));


            } else {

                drive_FL.setPower(0);
                drive_RL.setPower(0);
                drive_FR.setPower(0);
                drive_RR.setPower(0);

            }

            if (degreesBore(rightArm.getCurrentPosition()) < degreesBore(angle) * 20) {

                rightArm.setPower(.6);
                leftArm.setPower(.6);

            } else {

                rightArm.setPower(0);
                leftArm.setPower(0);

            }

            telemetry.addData("Current Angle", degreesBore(rightArm.getCurrentPosition()));
            telemetry.addData("Target Degrees", degreesBore(angle) * 20);
            telemetry.update();
            currentDistance = RL_distance.getDistance(DistanceUnit.INCH);
        }

        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);
        rightArm.setPower(0);
        leftArm.setPower(0);

    }

    public void runBackwardsEncoder(double speed, double inputInches) {
        double encoderValue = inchesBore(inputInches);
        double startingValue = drive_FL.getCurrentPosition();

        while (drive_FL.getCurrentPosition() > startingValue - abs(encoderValue)) {
            if (drive_FL.getCurrentPosition() > startingValue - abs(encoderValue)) {

                drive_FL.setPower(speed);
                drive_RL.setPower(speed);
                drive_FR.setPower(speed);
                drive_RR.setPower(speed);

            } else {

                drive_FL.setPower(0);
                drive_RL.setPower(0);
                drive_FR.setPower(0);
                drive_RR.setPower(0);

            }
        }
    }

    public void runBackwardsEncoderAndRaiseArm(double speed, double inputInches, int angle) {
        double targetValue = inchesBore(inputInches);
        double startingValue = drive_FL.getCurrentPosition();

        while (drive_FL.getCurrentPosition() > startingValue - abs(targetValue) || degreesBore(rightArm.getCurrentPosition()) < degreesBore(angle) * 20) {

            if (drive_FL.getCurrentPosition() > startingValue - abs(targetValue)) {

                drive_FL.setPower(speed);
                drive_RL.setPower(speed);
                drive_FR.setPower(speed);
                drive_RR.setPower(speed);

            } else {

                drive_FL.setPower(0);
                drive_RL.setPower(0);
                drive_FR.setPower(0);
                drive_RR.setPower(0);

            }

            if (degreesBore(rightArm.getCurrentPosition()) < degreesBore(angle) * 20) {

                rightArm.setPower(.3);
                leftArm.setPower(.3);

            } else {

                rightArm.setPower(0);
                leftArm.setPower(0);

            }

            telemetry.addData("Current Encoder",  drive_FL.getCurrentPosition());
            telemetry.addData("Target Degrees", startingValue - abs(targetValue));
            telemetry.update();

        }

        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);
        rightArm.setPower(0);
        leftArm.setPower(0);

    }

    public void strafeDiagonalDownLeftEncoderAndRaiseArm(double speed, double inputInches, int angle) {
        double encoderValue = inchesBore(inputInches);
        double startingValue = drive_FL.getCurrentPosition();

        while (drive_FL.getCurrentPosition() > startingValue - abs(encoderValue) || degreesBore(rightArm.getCurrentPosition()) < degreesBore(angle) * 20) {

            if (drive_FL.getCurrentPosition() > startingValue - abs(encoderValue)) {

                //drive_FL.setPower(speed);
                // drive_RL.setPower(speed - .3);// wasnt powered before -Nolan 4/4/22
                //drive_FR.setPower(speed - .3);
                // drive_RR.setPower(speed);


                driveTrain(speed, speed - .3, speed - .3, speed, 1);
            } else {

                drive_FL.setPower(0);
                drive_RR.setPower(0);
                drive_FR.setPower(0);//wasnt unpowered before -Nolan 4/4/22
                drive_RL.setPower(0);//same with this one

            }

            if (degreesBore(rightArm.getCurrentPosition()) < degreesBore(angle) * 20) {

                rightArm.setPower(.6);
                leftArm.setPower(.6);

            } else {

                rightArm.setPower(0);
                leftArm.setPower(0);

            }

            telemetry.addData("Current Encoder",  drive_FL.getCurrentPosition());
            telemetry.addData("Target Degrees", startingValue - abs(encoderValue));
            telemetry.update();

        }

        drive_FL.setPower(0);
        drive_RR.setPower(0);
        drive_FR.setPower(0);
        rightArm.setPower(0);
        leftArm.setPower(0);

    }

    public void strafeRightEncoder(double speed, double inputInches) {

        double encoderValue = abs(inchesBore(inputInches));

        double startingValue = abs(drive_RL.getCurrentPosition());
        double targetValue = abs(startingValue) + abs(encoderValue);
        double currentValue = startingValue;
        while (currentValue < targetValue) {

           // drive_FL.setPower(-speed);
           // drive_RL.setPower(speed);
            //drive_FR.setPower(speed);
            //drive_RR.setPower(-speed);
            driveTrain(-speed, speed, speed, -speed, PIDadjustment(1/inchesBore(2), targetValue, currentValue));

            telemetry.addData("Encoder Target", abs(drive_RL.getCurrentPosition()));
            telemetry.addData("Back Dead Encoder Running", targetValue);
            telemetry.update();
            currentValue = abs(drive_RL.getCurrentPosition());

        }

        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);

        telemetry.addData("Encoder Target", encoderValue);
        telemetry.addData("Back Dead Encoder Finished", drive_RL.getCurrentPosition());
        telemetry.update();

    }

    public int readDisVision() {

        if (frontDistanceLeft.getDistance(DistanceUnit.INCH) < 20) {

            return 2;

        } else if (frontDistanceRight.getDistance(DistanceUnit.INCH) < 20) {

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

            return 2;

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

        while (error > .3) {

            error = RL_distance.getDistance(DistanceUnit.INCH) - RR_distance.getDistance(DistanceUnit.INCH);

            drive_FL.setPower(0.15);
            drive_RL.setPower(0.15);
            drive_FR.setPower(-0.15);
            drive_RR.setPower(-0.15);
            telemetry.addData("Error", error);
            telemetry.addData("Left Distance", RL_distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Left Distance", RR_distance.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }

        while (error < -.3) {

            error = RL_distance.getDistance(DistanceUnit.INCH) - RR_distance.getDistance(DistanceUnit.INCH);

            drive_FL.setPower(-0.15);
            drive_RL.setPower(-0.15);
            drive_FR.setPower(0.15);
            drive_RR.setPower(0.15);
            telemetry.addData("Error", error);
            telemetry.addData("Left Distance", RL_distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Left Distance", RR_distance.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }

        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);

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

    public void strafeLeft(double speed, double inches) {
        double currentDistance;


        currentDistance = LS_distance.getDistance(DistanceUnit.INCH);
        while (LS_distance.getDistance(DistanceUnit.INCH) > inches) {
           // drive_FL.setPower(speed);
           // drive_RL.setPower(-speed);
           // drive_FR.setPower(-speed);
           // drive_RR.setPower(speed);
            driveTrain(speed, -speed, -speed, speed, PIDadjustment(0.5, currentDistance, inches));
           // driveTrain(speed, -speed, -speed, speed, 1);
            currentDistance = LS_distance.getDistance(DistanceUnit.INCH);
        }
        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);
    }

    public void strafeLeftTimed(double speed, double inches) {

        runtime.reset();

        while (LS_distance.getDistance(DistanceUnit.INCH) > inches) {
            if (LS_distance.getDistance(DistanceUnit.INCH) > inches && opModeIsActive() && runtime.seconds() <= 1) {
                drive_FL.setPower(speed);
                drive_RL.setPower(-speed);
                drive_FR.setPower(-speed);
                drive_RR.setPower(speed);
            }
            drive_FL.setPower(0);
            drive_RL.setPower(0);
            drive_FR.setPower(0);
            drive_RR.setPower(0);

            telemetry.addData("Left Side Distance",LS_distance.getDistance(DistanceUnit.INCH));
        }
    }
//change distance sensor to odometry in future
    public void strafeLeftArmRaise(double speed, double inches, int angle) {
        while (LS_distance.getDistance(DistanceUnit.INCH) > inches + 4 || degreesBore(rightArm.getCurrentPosition()) < degreesBore(angle) * 20) {

            if (LS_distance.getDistance(DistanceUnit.INCH) > inches + 4) {
                drive_FL.setPower(speed);
                drive_RL.setPower(-speed);
                drive_FR.setPower(-speed);
                drive_RR.setPower(speed);
            } else {
                drive_FL.setPower(0);
                drive_RL.setPower(0);
                drive_FR.setPower(0);
                drive_RR.setPower(0);
            }
            if (degreesBore(rightArm.getCurrentPosition()) < degreesBore(angle) * 20) {
                rightArm.setPower(.6);
                leftArm.setPower(.6);
            } else {
                rightArm.setPower(0);
                leftArm.setPower(0);
            }

            telemetry.addData("Current Angle", degreesBore(rightArm.getCurrentPosition()));
            telemetry.addData("Target Degrees", degreesBore(angle) * 20);
            telemetry.update();

        }
        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);
        rightArm.setPower(0);
        leftArm.setPower(0);

    }

    public void strafeLeftFromWall(double inches) {
        while (LS_distance.getDistance(DistanceUnit.INCH) < inches + 4) {
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

    public void strafeRightFromWall(double inches) {
        while (LS_distance.getDistance(DistanceUnit.INCH) < inches + 4) {
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

    public void forwardsDistanceDrive(double speed, int inches) {
        double currentDistance;

        currentDistance = RL_distance.getDistance(DistanceUnit.INCH);
        while (currentDistance < inches) {
           // drive_FL.setPower(-speed);
           // drive_RL.setPower(-speed);
           // drive_FR.setPower(-speed);
           // drive_RR.setPower(-speed);
           driveTrain(-speed, -speed, -speed, -speed, PIDadjustment(0.5, inches, currentDistance));
            currentDistance = RL_distance.getDistance(DistanceUnit.INCH);
        }
            drive_FL.setPower(0);
            drive_RL.setPower(0);
            drive_FR.setPower(0);
            drive_RR.setPower(0);
    }

    public void forwardsDistanceDriveFront(int inches) {
        while (frontDistanceLeft.getDistance(DistanceUnit.INCH) > inches) {
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

    public void backwardsDistanceDrive(double speed, int inches) {
        double currentDistance;

        currentDistance = RL_distance.getDistance(DistanceUnit.INCH);
        while (currentDistance > inches) {
            //drive_FL.setPower(speed);
            //drive_RL.setPower(speed);
            //drive_FR.setPower(speed);
            //drive_RR.setPower(speed);
            driveTrain(speed, speed, speed, speed, PIDadjustment(0.5, currentDistance, inches));
            currentDistance = RL_distance.getDistance(DistanceUnit.INCH);
        }
        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);
    }

    public void forwardsDistanceHub(int inches) {
        if (frontDistanceLeft.getDistance(DistanceUnit.INCH) > inches) {
            drive_FL.setPower(-.3);
            drive_RL.setPower(-.3);
            drive_FR.setPower(-.3);
            drive_RR.setPower(-.3);
            telemetry.addData("frontDistance", String.format("%.01f in", frontDistanceLeft.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        } else {
            drive_FL.setPower(0);
            drive_RL.setPower(0);
            drive_FR.setPower(0);
            drive_RR.setPower(0);
            telemetry.addData("frontDistance", String.format("%.01f in", frontDistanceLeft.getDistance(DistanceUnit.INCH)));
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

    public void turnRightArmRaise(double degrees, int angle){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2 || degreesBore(rightArm.getCurrentPosition()) < degreesBore(angle) * 20) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - abs(getAngle());
            telemetry.addData("error", error);
            telemetry.update();
        } if (degreesBore(rightArm.getCurrentPosition()) < degreesBore(angle) * 20) {

            rightArm.setPower(.6);
            leftArm.setPower(.6);

        } else {

            rightArm.setPower(0);
            leftArm.setPower(0);

        }

        telemetry.addData("Current Angle", degreesBore(rightArm.getCurrentPosition()));
        telemetry.addData("Target Degrees", degreesBore(angle) * 20);
        telemetry.update();





        drive_FL.setPower(0);
        drive_RL.setPower(0);
        drive_FR.setPower(0);
        drive_RR.setPower(0);
    }

    public void turnRight(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.7 : 0.7); //.5 before
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
            double motorPower = (error < 0 ? -0.7 : 0.7); //.5 before
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
    public double PIDadjustment(double kP, double targetValue, double sensorValue) {

        return((targetValue - sensorValue) * kP);

    }

    public void driveTrain(double FLSpeed, double RLSpeed, double FRSpeed, double RRSpeed, double PIDScale) {

        double maxSpeed = 0;

        FLSpeed = (FLSpeed * PIDScale);
        RLSpeed = (RLSpeed * PIDScale);
        FRSpeed = (FRSpeed * PIDScale);
        RRSpeed = (RRSpeed * PIDScale);

        //maxSpeed = Math.max(abs(FLSpeed), abs(RLSpeed), abs(FRSpeed), abs(RRSpeed));
        if (abs(FLSpeed) > maxSpeed) {

            maxSpeed = abs(FLSpeed);
        }
        else if  (abs(RLSpeed) > maxSpeed) {

            maxSpeed = abs(RLSpeed);
        }
        else if  (abs(FRSpeed) > maxSpeed) {

            maxSpeed = abs(FRSpeed);
        }
        else if  (abs(RRSpeed) > maxSpeed) {

            maxSpeed = abs(RRSpeed);
        }
        if (abs(maxSpeed) > 1) {
            drive_FL.setPower(FLSpeed / maxSpeed);
            drive_RL.setPower(RLSpeed / maxSpeed);
            drive_FR.setPower(FRSpeed / maxSpeed);
            drive_RR.setPower(RRSpeed / maxSpeed);
        }
        else {
            drive_FL.setPower(FLSpeed);
            drive_RL.setPower(RLSpeed);
            drive_FR.setPower(FRSpeed);
            drive_RR.setPower(RRSpeed);
        }
    }

}