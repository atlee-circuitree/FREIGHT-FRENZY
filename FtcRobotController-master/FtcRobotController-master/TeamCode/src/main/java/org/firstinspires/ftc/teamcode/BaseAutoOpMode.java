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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public void ResetDriveEncoder(){
        SetDriveMode(Mode.STOP_RESET_ENCODER);
        SetDriveMode(Mode.RUN_WITH_ENCODER);
    }


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

    public void SimpleRotate(double angle, double speed) {

        yawPIDController.setInputRange(0, 360);

        targetAngle = navx_centered.getYaw() + angle;

        if (navx_centered.getYaw() >= targetAngle) {

            while (navx_centered.getYaw() >= targetAngle) {

                drive_FL.setPower(speed);
                drive_FR.setPower(-speed);
                drive_RL.setPower(speed);
                drive_RR.setPower(-speed);

            }

        }

        if (navx_centered.getYaw() <= targetAngle) {

            while (navx_centered.getYaw() <= targetAngle) {

                drive_FL.setPower(-speed);
                drive_FR.setPower(speed);
                drive_RL.setPower(-speed);
                drive_RR.setPower(speed);

            }

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

        final double toleranceDegrees = 5.0;
        final double P = 0.02;
        final double I = 0.00;
        final double D = 0.00;

        yawPIDController.setPID(P,I,D);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, toleranceDegrees);
        yawPIDController.setContinuous(true);
        yawPIDController.setInputRange(-180, 180);
        yawPIDController.setInputRange(-.5, .5);
        yawPIDController.setSetpoint(targetDegrees);
        yawPIDController.enable(true);



        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
            while (opModeIsActive()) {

                navXPIDController.PIDResult PIDResult = new navXPIDController.PIDResult();

                double output = PIDResult.getOutput();

                if (navx_centered.getYaw() < 0) {


                }

                if (PIDResult.isOnTarget()) {
                    telemetry.addData("PID Output", "On target");
                    drive_FL.setPower(0);
                    drive_FR.setPower(0);
                    drive_RL.setPower(0);
                    drive_RR.setPower(0);
                }
                else {
                    telemetry.addData("PID Output", PIDResult.getOutput());
                    telemetry.addData("NavX Yaw", navx_centered.getYaw());
                    drive_FL.setPower(speed);
                    drive_FR.setPower(-speed);
                    drive_RL.setPower(speed);
                    drive_RR.setPower(-speed);
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
}












