
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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Mecanum Wheel Drive", group="Opmode")
public class Mecanum_Wheel_Program extends NemoBot
{
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     * Called from parent class: Ember2Bot
     */
    @Override
    public void nemoLoop() {
        //Note: super is calling rom parent class
        super.nemoLoop();


        boolean intakeButton = gamepad2.right_bumper;
        boolean reverseIntakeButton = gamepad2.left_bumper;
        // Using 0.1 as threshold for button to avoid accidental trigger
        boolean ejectButton = gamepad2.right_trigger > 0.1;
        boolean reverseEjectButton = gamepad2.left_trigger > 0.1;

        boolean servoUp = gamepad2.dpad_up;
        boolean servoDown = gamepad2.dpad_down;

        //Taking values from the gamepad
        mecanumDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);

        if (intakeButton) {
            intake.setPower(1);
            middle.setPower(1);
        } else if (reverseIntakeButton) {
            intake.setPower(-1);
            middle.setPower(-1);
        } else {
            intake.setPower(0);
            middle.setPower(0);
        }

        if(ejectButton) {
            middle.setPower(1);
            eject.setPower(1);
        } else if (reverseEjectButton) {
            middle.setPower(-1);
            eject.setPower(-1);
        } else {
            middle.setPower(0);
            eject.setPower(0);
        }

        if(servoUp) {
            grabLeft.setPosition(0);
            grabRight.setPosition(0);
        } else if (servoDown) {
            grabLeft.setPosition(0.75);
            grabRight.setPosition(0.75);
        }

        // Show the elapsed game time and wheel power & updating telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("gamepad", "1: left.x (%.2f), left.y (%.2f), right.x (%.2f), right.y (%.2f)",
                gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.addData("gamepad", "2: left.x (%.2f), left.y (%.2f), right.x (%.2f), right.y (%.2f)",
                gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.right_stick_y);
        telemetry.addData("Motors", "m0 (%.2f), m1 (%.2f), m2 (%.2f), m3 (%.2f)", v0, v1, v2, v2);
        telemetry.update();
    }



}
