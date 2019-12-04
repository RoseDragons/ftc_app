
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

@Autonomous(name="Ember2Depot", group="Opmode")
@Disabled
public class Ember2Depot extends Ember2Auto {

    /*
     * Code to run ONCE when the driver hits PLAY
     * Add extra code when on Depot side after Gold is moved
     */
    @Override
    public void emberStart() {
        super.emberStart();

        // Move back to not hit other minerals while turning
        mecanumDriveForMilliSec(0, 0, 0, -0.8, 650);

        //Turn towards depot
        switch (goldPosition) {
            case LEFT:
                telemetry.addData("Turn", "Left");
                telemetry.update();
                turnToAngle(0.8, -60);
                // Go forward
                mecanumDriveForMilliSec(0, 0, 0, -0.7, 1200);
                // Turn right
                turnToAngle(0.8, -125);
                // Into depot
                mecanumDriveForMilliSec(0, 0, 0, -0.9, 900);
                break;
            case CENTER:
                telemetry.addData("Turn", "Center");
                telemetry.update();
                turnToAngle(0.8, -90);
                // Go forward
                mecanumDriveForMilliSec(0, 0, 0, -0.8, 1650);
                break;
            case RIGHT:
                telemetry.addData("Turn", "Right");
                telemetry.update();
                // Turn right
                turnToAngle(0.8, -110);
                // Go forward
                mecanumDriveForMilliSec(0, 0, 0, -0.8, 1200);
                // Turn left
                turnToAngle(0.8, -60);
                // Into depot
                mecanumDriveForMilliSec(0, 0, 0, -0.9, 800);
                break;
        }


        //Drop the Marker
        DragonDrop.setPosition(0.75);

        // Stop
        mecanumDriveForMilliSec(0, 0, 0, 0, 300);

        // Clear depot
        switch (goldPosition) {
            case LEFT:
                turnToAngle(0.8, -145);
                mecanumDriveForMilliSec(0, 0, 0, 0.95, 800);
                break;
            case CENTER:
                turnToAngle(0.8, -152);
                mecanumDriveForMilliSec(0, 0, 0, 0.95, 825);
                break;
            case RIGHT:
                turnToAngle(0.8, -125);
                mecanumDriveForMilliSec(0, 0, 0, 0.95, 300);
                break;
        }

        // Go to opponent crater
        turnToAngle(0.7, -120);
        mecanumDriveForMilliSec(0, 0, 0, 0.95, 2000);

        // Stop
        mecanumDriveForMilliSec(0, 0, 0, 0, 25);
    }
}