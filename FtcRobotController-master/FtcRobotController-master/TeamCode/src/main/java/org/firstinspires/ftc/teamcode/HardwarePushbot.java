package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot
{
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    /* Public OpMode members. */
    public DcMotor drive_FL;
    public DcMotor drive_RL;
    public DcMotor drive_FR;
    public DcMotor drive_RR;

    public BNO055IMU imu;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        drive_FL   = hwMap.get(DcMotor.class, "drive_FL");
        drive_FR  = hwMap.get(DcMotor.class, "drive_FR");
        drive_RL   = hwMap.get(DcMotor.class, "drive_RL");
        drive_RR  = hwMap.get(DcMotor.class, "drive_RR");
        drive_FL.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        drive_FR.setDirection(DcMotor.Direction.FORWARD);
        drive_RL.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        drive_RR.setDirection(DcMotor.Direction.FORWARD);
        drive_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive_RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        setAllPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        drive_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive_RL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive_RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //Set power to all motors
    public void setAllPower(double p){
        setMotorPower(p,p,p,p);
    }

    public void setMotorPower(double lF, double rF, double lB, double rB){
        drive_FL.setPower(lF);
        drive_RL.setPower(lB);
        drive_FR.setPower(rF);
        drive_RR.setPower(rB);
    }
}