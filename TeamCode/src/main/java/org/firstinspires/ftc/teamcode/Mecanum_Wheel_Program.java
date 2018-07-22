
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum Wheel Drive", group="Linear Opmode")
//@Disabled
public class Mecanum_Wheel_Program extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;
    DcMotor Motor_0;
    DcMotor Motor_1;
    DcMotor Motor_2;
    DcMotor Motor_3;

    double v0;
    double v1;
    double v2;
    double v3;

    //double power = 0.5;
    //double leftPower = 0.5;
    //double rightPower = -0.5;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Setup a variable for each drive wheel to save power level for telemetry
        telemetry.addData("Status", "Running: " + runtime.toString());
        Motor_0 = hardwareMap.get(DcMotor.class, "Motor_0");
        Motor_1 = hardwareMap.get(DcMotor.class, "Motor_1");
        Motor_2 = hardwareMap.get(DcMotor.class, "Motor_2");
        Motor_3 = hardwareMap.get(DcMotor.class, "Motor_3");


        Motor_0.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_1.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_2.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_3.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void waitForStart() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        telemetry.addData("Status", "Initialized");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;
        //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        //The robot goes forward, backward, left and right on using only one stick.
        //double drive = -gamepad1.right_stick_y;
        //double turn = gamepad1.right_stick_x ;

        //leftPower = Range.clip(drive + turn, -1, 1);
        //rightPower = Range.clip(drive - turn, -1, 1);

        //Setting power to motors using trigonometric algorithm for mecanum wheels
        double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_x, gamepad1.right_stick_y) - (Math.PI / 4);
        double rightX = gamepad1.left_stick_x;
        v0 = r * Math.cos(robotAngle) + rightX;
        v1 = r * Math.sin(robotAngle) + rightX;
        v2 = r * Math.sin(robotAngle) - rightX;
        v3 = r * Math.cos(robotAngle) - rightX;

        Motor_0.setPower(Range.clip(v0, -1, 1));
        Motor_1.setPower(Range.clip(v1, -1, 1));
        Motor_2.setPower(Range.clip(v2, -1, 1));
        Motor_3.setPower(Range.clip(v3, -1, 1));

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("gamepad", "1: right.x (%.2f), right.y (%.2f), left.x (%.2f), left.y (%.2f)",
                gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.addData("Motors", "m0 (%.2f), m1 (%.2f), m2 (%.2f), m3 (%.2f)", v0, v1, v2, v2);
    }

    /*
    //public final void sleep(long milliseconds) {
        //try {
            //Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    */

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        Motor_0.setPower(0);
        Motor_1.setPower(0);
        Motor_2.setPower(0);
        Motor_3.setPower(0);
    }

}
