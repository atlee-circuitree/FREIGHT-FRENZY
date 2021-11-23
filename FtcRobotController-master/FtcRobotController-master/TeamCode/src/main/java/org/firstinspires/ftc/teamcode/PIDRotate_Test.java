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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="PIDRotate_test", group="Pushbot")

public class PIDRotate_Test extends BaseAutoOpMode {

    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        GetHardware();

        waitForStart();

        SimpleRotateLeft(-90, .25);

    }

    public void SimpleRotateLeft(double angle, double speed) {

        // Sets the range between -180 and 180
        yawPIDController.setInputRange(-180, 180);

        // Creates a target angle
        double targetAngle = navx_centered.getYaw() + angle;

        if (navx_centered.getYaw() >= targetAngle) {

            while (navx_centered.getYaw() >= targetAngle) {

                // Turns Left
                drive_FL.setPower(speed);
                drive_FR.setPower(-speed);
                drive_RL.setPower(speed);
                drive_RR.setPower(-speed);

                telemetry.addData("Target Angle ", targetAngle);
                telemetry.addData("Current Angle ", navx_centered.getYaw());
                telemetry.update();

            }

        }

        drive_FL.setPower(0);
        drive_FR.setPower(0);
        drive_RL.setPower(0);
        drive_RR.setPower(0);

        telemetry.addData("Target Angle ", targetAngle);
        telemetry.addData("Current Angle ", navx_centered.getYaw());
        telemetry.update();
    }
}
