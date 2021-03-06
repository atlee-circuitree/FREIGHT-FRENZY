/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import android.util.Log;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.kauailabs.navx.ftc.navXPIDController;

/**
 * This file contains basic code to run a 4 wheeled Mecanum wheel setup. The d-pad controls
 * forwards/backwards and turning left and right, and the right stick controls strafing. (working on diff. control setup currently)
 */
@Disabled
public abstract class BaseAutoOpMode extends BaseOpMode {


    public navXPIDController yawPIDController;



    @Override

    public void GetHardware() {
        super.GetHardware();
        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        yawPIDController = new navXPIDController( navx_centered, navXPIDController.navXTimestampedDataSource.YAW);


    }

    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware

    static final double     FORWARD_SPEED = 1;
    static final double     TURN_SPEED    = 1;

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    public void encoderStrafeV4( double speed, double distance, double timeout) {
        int RevEncoderTarget;

        if (opModeIsActive()) {
            RevEncoderTarget = drive_FL.getCurrentPosition() + (int) (distance * OMNI_COUNTS_PER_INCH);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < timeout) && (drive_FL.getCurrentPosition() < RevEncoderTarget)) {
                drive_FL.setPower(-Math.abs(speed));
                drive_FR.setPower(Math.abs(speed));
                drive_RL.setPower(Math.abs(speed));
                drive_RR.setPower(-Math.abs(speed));
                telemetry.addData("Path3", "Running to %7d", RevEncoderTarget);
                telemetry.addData("Path4", "Running at %7d", drive_FL.getCurrentPosition());
                telemetry.update();
            }
            DriveTrain(Drive.STOP);
            while (opModeIsActive() && (runtime.seconds() < timeout) && (drive_FL.getCurrentPosition() > RevEncoderTarget)) {
                drive_FL.setPower(Math.abs(speed));
                drive_FR.setPower(-Math.abs(speed));
                drive_RL.setPower(-Math.abs(speed));
                drive_RR.setPower(Math.abs(speed));
                telemetry.addData("Path3", "Running to %7d", RevEncoderTarget);
                telemetry.addData("Path4", "Running at %7d", drive_FL.getCurrentPosition());
                telemetry.update();
            }
            DriveTrain(Drive.STOP);

            //  SetDriveMode(Mode.RUN_WITH_ENCODER);
        }
    }


    public void encoderStrafeV5(double speed, double distance, double timeout) {

        int RevEncoderTarget;

        if (opModeIsActive()) {
            RevEncoderTarget = drive_FL.getCurrentPosition() + (int) (distance * OMNI_COUNTS_PER_INCH);

            runtime.reset();
            if(distance > 0) {
                while (opModeIsActive() && (runtime.seconds() < timeout) && (drive_FL.getCurrentPosition() < RevEncoderTarget)) {
                    drive_FL.setPower(-Math.abs(speed));
                    drive_FR.setPower(Math.abs(speed));
                    drive_RL.setPower(Math.abs(speed));
                    drive_RR.setPower(-Math.abs(speed));
                    telemetry.addData("Path3", "Running to %7d", RevEncoderTarget);
                    telemetry.addData("Path4", "Running at %7d", drive_FL.getCurrentPosition());
                    telemetry.update();
                }
            }
            else {
                while (opModeIsActive() && (runtime.seconds() < timeout) && (drive_FL.getCurrentPosition() > RevEncoderTarget)) {
                    drive_FL.setPower(Math.abs(speed));
                    drive_FR.setPower(-Math.abs(speed));
                    drive_RL.setPower(-Math.abs(speed));
                    drive_RR.setPower(Math.abs(speed));
                    telemetry.addData("Path3", "Running to %7d", RevEncoderTarget);
                    telemetry.addData("Path4", "Running at %7d", drive_FL.getCurrentPosition());
                    telemetry.update();
                }
            }
            DriveTrain(Drive.STOP);

        }
    }




    public void encoderDrive(double speed, double distance, double timeoutS) {
        int newFLTarget;
        int newRLTarget;
        int newRRTarget;
        int newFRTarget;
        double rampSpeed;
        rampSpeed = speed / 50;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLTarget = drive_FL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newFRTarget = drive_FR.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRRTarget = drive_RR.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRLTarget = drive_RL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            drive_FL.setTargetPosition(newFLTarget);
            drive_FR.setTargetPosition(newFRTarget);
            drive_RL.setTargetPosition(newRLTarget);
            drive_RR.setTargetPosition(newRRTarget);

            // Turn On RUN_TO_POSITION
            drive_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive_RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive_RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            drive_FL.setPower(Math.abs(rampSpeed));
            drive_FR.setPower(Math.abs(rampSpeed));
            drive_RL.setPower(Math.abs(rampSpeed));
            drive_RR.setPower(Math.abs(rampSpeed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (drive_RL.isBusy() && drive_FL.isBusy() && drive_FR.isBusy() && drive_RR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newRLTarget, newFLTarget, newRRTarget, newFRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", drive_FL.getCurrentPosition(), drive_FR.getCurrentPosition(), drive_RL.getCurrentPosition(), drive_RR.getCurrentPosition());
                telemetry.update();

                drive_FL.setPower(Math.abs(rampSpeed));
                drive_FR.setPower(Math.abs(rampSpeed));
                drive_RL.setPower(Math.abs(rampSpeed));
                drive_RR.setPower(Math.abs(rampSpeed));

                if(distance > 0 && drive_FL.getCurrentPosition() > (newFLTarget/100)*50){
                    rampSpeed = rampSpeed - 4 * (speed/50);
                    telemetry.addData("rampSpeed", rampSpeed);
                }
                else if(distance < 0 && drive_FL.getCurrentPosition() < (newFLTarget/100)*50){
                    rampSpeed = rampSpeed - 4 * (speed/50);
                    telemetry.addData("rampSpeed", rampSpeed);
                }
                else if (rampSpeed < speed) {
                    rampSpeed = rampSpeed + 2 * (speed / 50);
                    telemetry.addData("rampSpeed",rampSpeed);
                }
            }
            // Stop all motion;
            DriveTrain(Drive.STOP);

            // Turn off RUN_TO_POSITION
            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);
            //  sleep(250);   // optional pause after each move

        }
    }


    public void encoderDriveNoRamp(double speed, double distance, double timeoutS) {
        int newFLTarget;
        int newRLTarget;
        int newRRTarget;
        int newFRTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLTarget = drive_FL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newFRTarget = drive_FR.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRRTarget = drive_RR.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRLTarget = drive_RL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            drive_FL.setTargetPosition(newFLTarget);
            drive_FR.setTargetPosition(newFRTarget);
            drive_RL.setTargetPosition(newRLTarget);
            drive_RR.setTargetPosition(newRRTarget);

            // Turn On RUN_TO_POSITION
            drive_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive_RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive_RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            drive_FL.setPower(Math.abs(speed));
            drive_FR.setPower(Math.abs(speed));
            drive_RL.setPower(Math.abs(speed));
            drive_RR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() <= timeoutS) && (drive_RL.isBusy() && drive_FL.isBusy() && drive_FR.isBusy() && drive_RR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newRLTarget, newFLTarget, newRRTarget, newFRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", drive_FL.getCurrentPosition(), drive_FR.getCurrentPosition(), drive_RL.getCurrentPosition(), drive_RR.getCurrentPosition());
                telemetry.addData("Runtime", runtime.seconds());
                telemetry.update();

                drive_FL.setPower(Math.abs(speed));
                drive_FR.setPower(Math.abs(speed));
                drive_RL.setPower(Math.abs(speed));
                drive_RR.setPower(Math.abs(speed));

            }
            // Stop all motion;
            DriveTrain(Drive.STOP);

            // Turn off RUN_TO_POSITION
            SetDriveMode(Mode.RUN_WITHOUT_ENCODERS);
            //  sleep(250);   // optional pause after each move

        }
    }

    public void encoderPIDDrive(double speed, double distance, double targetAngle, double timeout) throws InterruptedException {

        final double TARGET_ANGLE_DEGREES = targetAngle;
        final double TOLERANCE_DEGREES = 1.0;
        final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
        final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
        final double YAW_PID_P = 0.005;
        final double YAW_PID_I = 0.0;
        final double YAW_PID_D = 0.0;
        ElapsedTime runtime = new ElapsedTime();

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        int newFLTarget;
        int newRLTarget;
        int newRRTarget;
        int newFRTarget;
        double rampSpeed = speed / 20;

        newFLTarget = drive_FL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newFRTarget = drive_FR.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newRRTarget = drive_RR.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newRLTarget = drive_RL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        drive_FL.setTargetPosition(newFLTarget);
        drive_FR.setTargetPosition(newFRTarget);
        drive_RL.setTargetPosition(newRLTarget);
        drive_RR.setTargetPosition(newRRTarget);

        // Turn On RUN_TO_POSITION
        drive_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive_RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive_RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while ((runtime.time() < timeout) && (drive_RL.isBusy() && drive_FL.isBusy() && drive_FR.isBusy() && drive_RR.isBusy()) && opModeIsActive()){
            if (yawPIDController.waitForNewUpdate(yawPIDResult, 500)) {

                if(distance > 0 && drive_FL.getCurrentPosition() > (newFLTarget/100)*50){
                    rampSpeed = rampSpeed - 4 * (speed/20);
                    telemetry.addData("rampSpeed", rampSpeed);
                }
                else if(distance < 0 && drive_FL.getCurrentPosition() < (newFLTarget/100)*50){
                    rampSpeed = rampSpeed - 4 * (speed/20);
                    telemetry.addData("rampSpeed", rampSpeed);
                }
                else if (rampSpeed < speed) {
                    rampSpeed = rampSpeed + 2 * (speed/20);
                    telemetry.addData("rampSpeed",rampSpeed);
                }
                if (yawPIDResult.isOnTarget()) {
                    drive_FL.setPower(rampSpeed);
                    drive_FR.setPower(rampSpeed);
                    drive_RL.setPower(rampSpeed);
                    drive_RR.setPower(rampSpeed);
                } else {
                    double output = yawPIDResult.getOutput()*2;
                    if (output < 0) {
                        /* Rotate Left */
                        drive_FL.setPower(rampSpeed - output);
                        drive_FR.setPower(rampSpeed + output);
                        drive_RL.setPower(rampSpeed - output);
                        drive_RR.setPower(rampSpeed + output);
                    } else {
                        /* Rotate Right */
                        drive_FL.setPower(rampSpeed + output);
                        drive_FR.setPower(rampSpeed - output);
                        drive_RL.setPower(rampSpeed + output);
                        drive_RR.setPower(rampSpeed - output);
                    }
                }
            } else {
                /* A timeout occurred */
                telemetry.addData("navXRotateToAnglePIDOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
        }
    }


    public void PIDRotate(double targetDegrees, double speed) throws InterruptedException{

        final double P = 0.0075;
        final double I = 0.00;
        final double D = 0.00;

        PIDControllerAlgorithm PID = new PIDControllerAlgorithm(P,I,D);

        PID.setPID(P,I,D);
        PID.setContinuous(true);
        PID.setInputRange(-180,180);
        PID.setOutputRange(-1,1);
        PID.setTolerance(1.0);
        PID.setInput(navx_centered.getYaw());
        PID.setSetpoint(targetDegrees);
        PID.enable();


        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
            while (opModeIsActive()) {

                double output = PID.performPID(navx_centered.getYaw());


                if(PID.onTarget()){
                    telemetry.addData("PID Output", "On target");
                    telemetry.addData("NavX Yaw", navx_centered.getYaw());
                    drive_FL.setPower(0);
                    drive_FR.setPower(0);
                    drive_RL.setPower(0);
                    drive_RR.setPower(0);
                }
                else{
                    telemetry.addData("PID Output", output);
                    telemetry.addData("NavX Yaw", navx_centered.getYaw());
                    drive_FL.setPower(-output);
                    drive_FR.setPower(output);
                    drive_RL.setPower(-output);
                    drive_RR.setPower(output);
                }
                telemetry.update();
            }

    }


    public void checkForTimeout(){

        if(timeout == true){
            while(opModeIsActive()){
                telemetry.addData("It timed out", "Did you turn the robot on/off?");
                telemetry.update();
                sleep(1000);
            }
        }

    }


    public void rotate(int degrees, double speed){

        double angle = navx_centered.getYaw();

        if(angle > degrees){
            while(angle >= degrees){
                telemetry.addData("Angle",angle);
                telemetry.update();
                angle = navx_centered.getYaw();
                drive_FL.setPower(-speed);
                drive_RL.setPower(-speed);
                drive_FR.setPower(speed);
                drive_RR.setPower(speed);
            }
        }
        else if(angle < degrees){
            while(angle <= degrees){
                telemetry.addData("Angle",angle);
                telemetry.update();
                angle = navx_centered.getYaw();
                drive_FL.setPower(speed);
                drive_RL.setPower(speed);
                drive_FR.setPower(-speed);
                drive_RR.setPower(-speed);
            }
        }
        drive_FL.setPower(0);
        drive_FR.setPower(0);
        drive_RL.setPower(0);
        drive_RR.setPower(0);
    }

    public void distanceDrive(int distance, String distanceSensor) {
        if (distanceSensor == "LS") {
            if (LS_distance.getDistance(DistanceUnit.INCH) < distance) {
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
        } else if (distanceSensor == "RS") {
            if (RS_distance.getDistance(DistanceUnit.INCH) < distance) {
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
        } else if (distanceSensor == "RL") {
            if (RL_distance.getDistance(DistanceUnit.INCH) < distance) {
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
        } else if(distanceSensor == "RR") {
            if (RR_distance.getDistance(DistanceUnit.INCH) < distance) {
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

    public void backDistanceDrive(int distance, String distanceSensor) {
        if (distanceSensor == "RL") {
            if (LS_distance.getDistance(DistanceUnit.INCH) < distance) {
                drive_FL.setPower(-1);
                drive_RL.setPower(-1);
                drive_FR.setPower(-1);
                drive_RR.setPower(-1);
            } else {
                drive_FL.setPower(0);
                drive_RL.setPower(0);
                drive_FR.setPower(0);
                drive_RR.setPower(0);
            }
        } else if (distanceSensor == "RR") {
            if (RR_distance.getDistance(DistanceUnit.INCH) < distance) {
                drive_FL.setPower(-1);
                drive_RL.setPower(-1);
                drive_FR.setPower(-1);
                drive_RR.setPower(-1);
            } else {
                drive_FL.setPower(0);
                drive_RL.setPower(0);
                drive_FR.setPower(0);
                drive_RR.setPower(0);
            }
        }
    }

    public void distanceDriveRight() {
        if (RL_distance.getDistance(DistanceUnit.INCH) < 28) {
            drive_FL.setPower(0.5);
            drive_RL.setPower(-0.5);
            drive_FR.setPower(-0.5);
            drive_RR.setPower(0.5);
        }
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

    public void Feeder() {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 3.0))
            feeder.setPower(.5);
    }

    public int degreesBore(int input) {

        final int COUNTS_PER_BORE_MOTOR_REV = 8192;    // eg: GOBUILDA Motor Encoder
        int COUNTS_TICKS_PER_REV_PER_DEGREE = (COUNTS_PER_BORE_MOTOR_REV) / 360;

        return COUNTS_TICKS_PER_REV_PER_DEGREE * input;
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

    public void compareBackSensors() {
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

    public void spinDuckyLeft(double speed) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= 4.0))
            leftDucky.setPower(speed);
        telemetry.addData("Left Ducky Wheel", runtime.seconds());
        telemetry.update();
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

    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        robot.setAllPower(0);
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

        turn(error);
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












