
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

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/* Tensor Flow is the Google Machine Learning platform for minerasl detection.
 * (Taken and modified from FTC example)*/

//Ember2Auto has all the common code between autonomous
public class Ember2Auto extends Ember2Bot {
    //Constants used for the Mineral Detection
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    protected static final int LEFT = -1;
    protected static final int RIGHT = 1;
    protected static final int CENTER = 0;

    //Vuforia Key
    private static final String VUFORIA_KEY = "AbWUeZr/////AAAAGeFdMmFhE054nlTo+3C3a7ovQcRzjVKW4ojh1WA1rArshva3w9sIz6YZIjVwM293mlFK+UhFMhhsGiqjBoTSPnJmZRGs8xYa7ZhJT53WYyrXHh6PJP58bzUFFG+4Jl9dYKSYFv7ly9vI7eonJYrB59eIUpv4tgZl917fuYBzYMqEZ8NW2402r/RRmISh/lK23+6ogoCPE344qBoUt+sbgCgfMy3BNTjWWkJv0Z2gZGcs6t3/Od1jpaIbL0gHWbhdnL/VcOkcLUnVIsE0lbTjxerbAr6eYMfjQU8MEXZ01afj1IY7dtAk6fSl1rK6kFUcL9CZu6zX6HxQ6in9TURFEjzqjM1DLvaG/VuYfKTE8m3+";
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    // Vuforia settings
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

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

    protected int goldPosition = -1;

    /**
     * Called from Parent class
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void emberInit() {
        super.emberInit();

        //Initializing Vuforia
        initVuforia();

        // Initializing Tensor Flow
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Activate Tensor Flow Object Detection.
        if (tfod != null) {
            tfod.activate();
        } else {
            telemetry.addData("Sorry!", "TensorFlow failed to start");
        }

        telemetry.update();

        // Initialize acctuator to find zero ???
        //initAcc();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void emberStart() {
        super.emberStart();

        //Lower the robot and unhinge from hook on lander
        unlatch();

        //detectPic();
        moveGoldMineral();
    }

    /*
     * Code to run ONCE when the driver hits STOP
     */
    @Override
    public void emberStop() {
        if (tfod != null) {
            //Stopping Tensor Flow
            tfod.shutdown();
        }

        super.emberStop();
    }

    /**
     * Simulating values as if receiving from gamepad in TeleOp
     * @param left_x Left joystick is used for strafing
     * @param left_y
     * @param right_x Right joystick (x-axis) is used to turn
     * @param right_y Right joystick (y-axis) is used to move forward & backward
     * @param ms Milliseconds (How long to move)
     */
    protected void mecanumDriveForMilliSec(double left_x, double left_y,
                                           double right_x, double right_y, int ms) {
        long startTime = System.currentTimeMillis();
        // Keep robot running for ms (milli seconds) amount.
        while((System.currentTimeMillis() - startTime <= ms)) {
            if (opModeIsActive() == false) {
                stopAllDrive();
                break;
            }

            mecanumDrive(left_x, left_y, right_x, right_y);

            // Show gamepad input values and wheel power.
            telemetry.addData("gamepad", "1: left.x (%.2f), left.y (%.2f), right.x (%.2f), right.y (%.2f)",
                    left_x, left_y, right_x, right_x);
            telemetry.addData("Motors", "m0 (%.2f), m1 (%.2f), m2 (%.2f), m3 (%.2f)", v0, v1, v2, v2);
            telemetry.update();
        }
    }

    /**
     *
     */
    private void unlatch() {
        // Lower the robot  ????
        //moveAccTicks(ACC_MOTOR_MAX_TICKS - 1100, 1.0);

        // Unlatch
        // Little forward
        mecanumDriveForMilliSec(0, 0, 0, -0.65, 300);
        // Turn left
        mecanumDriveForMilliSec(0, 0, -0.9, 0,500);
    }

    /**
     *
     */
    private void moveGoldMineral() {
        turnToViewMinerals();

        goldPosition = findGoldMineral2();
        telemetry.update();

        // Move Gold by moving backwards (phone side)
        switch(goldPosition){
            case LEFT:
                turnToAngle(0.7, 105);
                break;
            case CENTER:
                turnToAngle(0.7, 87);
                break;
            case RIGHT:
                turnToAngle(0.7, 63);
                break;
        }

        mecanumDriveForMilliSec(0, 0, 0, 0.65, 2100);

        telemetry.update();
        return;
    }

    // Taken from FTC sample
    private int findGoldMineral() {
        double startTime = runtime.milliseconds();

        while (opModeIsActive() && runtime.milliseconds() < startTime + (4 * 1000)) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getTop();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getTop();
                        } else {
                            silverMineral2X = (int) recognition.getTop();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            return LEFT;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            return RIGHT;
                        } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            return CENTER;
                        }
                    }
                }
            }
            telemetry.update();
        }

        // Just return any value
        return findGoldMineral2();
    }

    private int findGoldMineral2() {
        double startTime = runtime.milliseconds();

        while (opModeIsActive() && runtime.milliseconds() < startTime + (3 * 1000)) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        if (recognition.getTop() < 350) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            return LEFT;
                        }

                        if (recognition.getTop() < 700) {
                            telemetry.addData("Gold Mineral Position", "Center");
                            return CENTER;
                        }

                        telemetry.addData("Gold Mineral Position", "Right");
                        return RIGHT;
                    }
                    telemetry.addData("height", recognition.getWidth());
                    telemetry.addData("getImageHeight", recognition.getImageWidth());
                }
            }
            telemetry.update();
        }

        return LEFT;
    }

    /**
     * Turn robot after unlatching to face camera towards minerals
     * Move under lander to view all three minerals in camera
     */
    private void turnToViewMinerals() {
        // turn left and move forward to go under lander
        turnToAngle(0.75, 74);
        mecanumDriveForMilliSec(0, 0, 0, -0.5, 505);
        telemetry.update();

        stopAllDrive();
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

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.45;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}