
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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;

import java.util.List;

public class Ember2Auto extends Ember2Bot {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AbWUeZr/////AAAAGeFdMmFhE054nlTo+3C3a7ovQcRzjVKW4ojh1WA1rArshva3w9sIz6YZIjVwM293mlFK+UhFMhhsGiqjBoTSPnJmZRGs8xYa7ZhJT53WYyrXHh6PJP58bzUFFG+4Jl9dYKSYFv7ly9vI7eonJYrB59eIUpv4tgZl917fuYBzYMqEZ8NW2402r/RRmISh/lK23+6ogoCPE344qBoUt+sbgCgfMy3BNTjWWkJv0Z2gZGcs6t3/Od1jpaIbL0gHWbhdnL/VcOkcLUnVIsE0lbTjxerbAr6eYMfjQU8MEXZ01afj1IY7dtAk6fSl1rK6kFUcL9CZu6zX6HxQ6in9TURFEjzqjM1DLvaG/VuYfKTE8m3+";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void emberInit() {
        super.emberInit();

        // Initialize acctuator to find zero
        //initAcc();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        } else {
            telemetry.addData("Sorry!", "TensorFlow failed to start");
        }

        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void emberStart() {
        super.emberStart();

        unlatch();

        moveGoldMineral();
    }

    @Override
    public void emberStop() {
        if (tfod != null) {
            tfod.shutdown();
        }

        super.emberStop();
    }


    protected void mecanumDriveForMilliSec(double left_x, double left_y,
                                           double right_x, double right_y, int ms) {

        long startTime = System.currentTimeMillis();
        while(opModeIsActive() && (System.currentTimeMillis() - startTime <= ms)) {
            mecanumDrive(left_x, left_y, right_x, right_y);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("gamepad", "1: left.x (%.2f), left.y (%.2f), right.x (%.2f), right.y (%.2f)",
                    left_x, left_y, right_x, right_x);
            telemetry.addData("Motors", "m0 (%.2f), m1 (%.2f), m2 (%.2f), m3 (%.2f)", v0, v1, v2, v2);
            telemetry.addData("Acc", "pos: %d", AccMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    /**
     *
     */
    private void unlatch() {
        // Lower the robot
        moveAccTicks(ACC_MOTOR_MAX_TICKS - 800, 1.0);

        // Unlatch
        // Little forward
        mecanumDriveForMilliSec(0, 0, 0, -0.65, 300);
        // Turn left
        mecanumDriveForMilliSec(0, 0, -0.9, 0,640);
        // Move back
        mecanumDriveForMilliSec(0, 0, 0, .95,1200);
    }

    /**
     *
     */
    private void moveGoldMineral() {
        while (opModeIsActive()) {
            if (isGoldMineral()) {

                // Stop
                mecanumDriveForMilliSec(0, 0, 0, 0, 50);

                // Move Gold here ????
                return;
            }

            // Turn right
            mecanumDriveForMilliSec(0, 0, 0.5, 0,100);

            telemetry.update();
        }
    }

    /**
     *
     * @return
     */
    private boolean isGoldMineral() {
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            if (updatedRecognitions.size() == 1) {
                if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                    telemetry.addData("Gold Mineral", "Found");
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}