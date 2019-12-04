
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


@Autonomous(name="Ember2Crater", group="Opmode")
@Disabled
public class Ember2Crater extends Ember2Auto {

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void emberStart() {
        super.emberStart();

        ////////// Go drop Marker
        // Move back from Gold mineral
        mecanumDriveForMilliSec(0, 0, 0, -0.8, 780);

        // Face the wall
        turnToAngle(0.7, -8);

        // Go towards wall
        switch (goldPosition) {
            case LEFT:
                mecanumDriveForMilliSec(0, 0, 0, -0.8, 1555);
                break;
            case CENTER:
                mecanumDriveForMilliSec(0, 0, 0, -0.8, 1615);
                break;
            case RIGHT:
                mecanumDriveForMilliSec(0, 0, 0, -0.8, 1645);
                break;
        }

        // Go to Depot
        turnToAngle(0.7, 33);
        mecanumDriveForMilliSec(0, 0, 0, -0.9, 1900);

        // Stop
        mecanumDriveForMilliSec(0, 0, 0, 0, 25);

        //Drop the Marker
        DragonDrop.setPosition(0.75);

        // Stop
        mecanumDriveForMilliSec(0, 0, 0, 0, 500);

        // Come back to crater
        turnToAngle(0.7, 43);
        mecanumDriveForMilliSec(0, 0, 0, 0.95, 2800);

        // Stop
        mecanumDriveForMilliSec(0, 0, 0, 0, 25);
    }
}