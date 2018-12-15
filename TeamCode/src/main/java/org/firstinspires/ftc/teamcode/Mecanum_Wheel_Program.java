
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
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum Wheel Drive", group="Opmode")
public class Mecanum_Wheel_Program extends Ember2Bot
{
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     * Called from parent class: Ember2Bot
     */
    @Override
    public void emberLoop() {
        //Note: super is calling rom parent class
        super.emberLoop();

        //Moving acctuator with gamepad
        double AccPower = -gamepad2.right_stick_y;
        if (inAccRange(ACC_MOTOR_MAX_TICKS, (AccPower > 0))) {
            AccMotor.setPower(AccPower);
        } else {
            AccMotor.setPower(0);
        }


        double boxServoSpeed = gamepad2.left_stick_y;

        //Taking values from the gamepad
        mecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);

        // Show the elapsed game time and wheel power & updating telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("gamepad", "1: left.x (%.2f), left.y (%.2f), right.x (%.2f), right.y (%.2f)",
                gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.addData("gamepad", "2: left.x (%.2f), left.y (%.2f), right.x (%.2f), right.y (%.2f)",
                gamepad2.left_stick_x, gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.right_stick_y);
        telemetry.addData("Motors", "m0 (%.2f), m1 (%.2f), m2 (%.2f), m3 (%.2f)", v0, v1, v2, v2);
        telemetry.addData("Acc", "pos: %d", AccMotor.getCurrentPosition());
        telemetry.update();
    }



}
