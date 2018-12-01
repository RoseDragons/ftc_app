package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public abstract class Ember2Bot extends LinearOpMode {
    // The IMU (Internal Measurement Unit) sensor object
    BNO055IMU imu;

    // State used for updating telemetry for IMU sensor
    Orientation angles;
    Acceleration gravity;

    //Constants for continuous servos
    protected static final double CONTINUOUS_SERVO_STOP = 0.5;
    protected static final double CONTINUOUS_SERVO_FORWARD = 1.0;
    protected static final double CONTINUOUS_SERVO_REVERSE = 0.0;

    //Acctuator's maximum height without causing damage.
    protected static final int ACC_MOTOR_MAX_TICKS = 21000;

    protected ElapsedTime runtime = new ElapsedTime();

    //Four DC motors for mecanum wheels
    DcMotor Motor_0;
    DcMotor Motor_1;
    DcMotor Motor_2;
    DcMotor Motor_3;

    //TorqueNado Motor for Acctuator
    DcMotor AccMotor;
    //Touch Sensor to prevent damage to acctuator
    DigitalChannel AccTouch;

    //CRServo servo_5;
    //CRServo servo_3;
    Servo DragonDrop;

    //Velocity for each wheel
    double v0;
    double v1;
    double v2;
    double v3;

    /*We used a Linear OpMode to consecutively go through the init, and the teleOp and control
    the timing ourselves.
    We broke each step of the teleOp in separate functions below.*/
    @Override
    public void runOpMode() {
        //Function for when init is pressed. (driver presses INIT)
        emberInit();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // When the PLAY is pressed.
        emberStart();
        while (opModeIsActive()) {
            //When TeleOp is running
            emberLoop();
        }
        //When stop is pressed
        emberStop();
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void emberInit() {
        // Setup a variable for each drive wheel to save power level for telemetry
        telemetry.addData("Status", "Running: " + runtime.toString());
        Motor_0 = hardwareMap.get(DcMotor.class, "Motor_0");
        Motor_1 = hardwareMap.get(DcMotor.class, "Motor_1");
        Motor_2 = hardwareMap.get(DcMotor.class, "Motor_2");
        Motor_3 = hardwareMap.get(DcMotor.class, "Motor_3");

        //Wheels stop immediately when power is set to 0.
        Motor_0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Our robot doesn't have a defined front or back.
        Motor_0.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_1.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_2.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_3.setDirection(DcMotorSimple.Direction.REVERSE);

        //Setting up acctuator motor
        AccMotor = hardwareMap.get(DcMotor.class, "torqueNADO");
        AccMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AccMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Setting up lower stop button with a touch sensor
        AccTouch = hardwareMap.get(DigitalChannel.class, "AccButton");
        AccTouch.setMode(DigitalChannel.Mode.INPUT);

        //servo_3 = hardwareMap.get(CRServo.class, "Servo_3");
        //servo_5 = hardwareMap.get(CRServo.class, "Servo_5");
        DragonDrop = hardwareMap.get(Servo.class, "DragonDrop");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode

        //Changed to false because, we don't need to troubleshoot
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeIMUTelemetry();

        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void emberStart() {
        //Resetting time shown on app
        runtime.reset();

        telemetry.addData("Status", "Initialized");
    }

    /*
     * This method will be called repeatedly in a loop while this op mode is running
     * See implementations in Ember2Auto and Mecanum_Wheel_Program
     */
    public void emberLoop() {};

    /*
     * This method will be called when this op mode is first disabled
     */
    public void emberStop() {
        //Making sure all wheels are stopped
        stopAllDrive();

        //Acctuator motor is also stopped
        AccMotor.setPower(0);

        //servo_3.setPower(0);
        //servo_5.setPower(0);
    }

    //Wheel's motor power is set to 0
    public void stopAllDrive() {
        Motor_0.setPower(0);
        Motor_1.setPower(0);
        Motor_2.setPower(0);
        Motor_3.setPower(0);
    }

    //Bringing acctuator down until its lower limit is reached
    protected void initAcc() {
        // Move acctuator until button is press
        AccMotor.setPower(-1);

        //Keeps motor running until button is pressed
        while (AccTouch.getState() == true && opModeIsActive()) {
            telemetry.addData("accMotor", "pos: %d", AccMotor.getCurrentPosition());
            telemetry.update();
        }

        //Once button is hit, stop
        AccMotor.setPower(0);

        //When button is hit, acctuator position is at 0 position
        //Note: Use encoders to calculate ticks, not to run the motor.
        AccMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AccMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Moving acctuator up or down.
     * @param ticks  Number of ticks we want to move. To lower, use negative value and vise versa.
     * @param AccPower  Setting power to control speed
     */
    protected void moveAccTicks(int ticks, double AccPower) {
        AccMotor.setTargetPosition(ticks + AccMotor.getCurrentPosition());

        //To lower, run motor in reverse
        if (ticks < 0) {
            AccPower = -AccPower;
        }
        AccMotor.setPower(AccPower);

        //Waiting until motor reaches target and hasn't gone out of range
        while (opModeIsActive() &&
                isMotorBusy(AccMotor, ticks > 0) &&
                inAccRange(ACC_MOTOR_MAX_TICKS, ticks > 0)) {

            //Motor has power and is running during this loop
            telemetry.addData("accMotor", "pos: %d", AccMotor.getCurrentPosition());
            telemetry.addData("accMotor", "tartger pos: %d", AccMotor.getTargetPosition());
            telemetry.update();
        }

        //Stop Motor
        AccMotor.setPower(0.0);
    }

    //Checking if motor needs to keep running or if it has reached its target
    protected boolean isMotorBusy(DcMotor dcMotor, boolean isForward) {
        int targetPosition = dcMotor.getTargetPosition();
        int currentPosition = dcMotor.getCurrentPosition();

        //If robot is moving forward and motor has crossed the target, then motor should stop.
        if (isForward && currentPosition > targetPosition) {
            return false;
        }
        //If robot is moving backward and motor has passed lower than the target, then motor should stop.
        if (!isForward && currentPosition < targetPosition) {
            return false;
        }
        return true;
    }

    /**
     * To not allow acctuator to not cross upward.
     * @param max actuator max ticks
     * @param isForward Limit of acctuator when moving up
     */
    protected boolean inAccRange(int max, boolean isForward) {

        //If going up, check ticks value
        if (isForward) {
            return (AccMotor.getCurrentPosition() < max);

        } else {
            //Going down, check if button is pressed
            if (AccTouch.getState() == false) {

                // If button is pressed, we cannot go further down
                AccMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                AccMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                return false;
            }

            return true;
        }
    }

    /**
     * Main function for mecanum drive
     * Values coming from gamepad
     * @param left_x Left joystick is used for strafing
     * @param left_y
     * @param right_x Right joystick (x-axis) is used to turn
     * @param right_y Right joystick (y-axis) is used to move forward & backward
     */
    protected void mecanumDrive(double left_x, double left_y, double right_x, double right_y) {
        //Setting power to motors using trigonometric algorithm for mecanum wheels
        double r = Math.hypot(right_x, right_y);
        double robotAngle = Math.atan2(right_x, right_y) - (Math.PI / 4);
        double rightX = left_x;

        //Calculating velocity for each motor using polar coordinates
        v0 = r * Math.cos(robotAngle) + rightX;
        v1 = r * Math.sin(robotAngle) + rightX;
        v2 = r * Math.sin(robotAngle) - rightX;
        v3 = r * Math.cos(robotAngle) - rightX;

        //Setting values for motor speed
        Motor_0.setPower(Range.clip(v0, -1, 1));
        Motor_1.setPower(Range.clip(v1, -1, 1));
        Motor_2.setPower(Range.clip(v2, -1, 1));
        Motor_3.setPower(Range.clip(v3, -1, 1));
    }

    protected void turnDegrees (double power, float degrees) {
        Orientation orientation  = imu.getAngularOrientation();
        float startHeading = orientation.firstAngle;

        float targetHeading = startHeading + degrees;
        if (degrees > 0) {
            power = -power;
        }

        targetHeading = AngleUnit.normalizeDegrees(targetHeading);
        float currentHeading = startHeading;

        while(opModeIsActive() &&
                ((degrees > 0 && currentHeading < targetHeading) ||
                 (degrees < 0 && currentHeading > targetHeading))) {

            if (opModeIsActive() == false) {
                break;
            }

            mecanumDrive(0, 0, power, 0);

            telemetry.clearAll();
            telemetry.addData("Start Heading", "%.2f", startHeading);
            telemetry.addData("Target Heading", "%.2f", targetHeading);
            telemetry.addData("Current Heading", "%.2f", currentHeading);
            telemetry.update();

            currentHeading = imu.getAngularOrientation().firstAngle;
        }
        stopAllDrive();
    }

    public void pause(long ms) {
        try {
            stopAllDrive();
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    //----------------------------------------------------------------------------------------------
    // IMU Telemetry Configuration (From FTC Example)
    //----------------------------------------------------------------------------------------------
    void composeIMUTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addData("Status", new Func<String>() {
            @Override public String value() {
                return "Run Time: " + runtime.toString();
            }
        });
        telemetry.addLine()
                .addData("IMU status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        /*telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });*/
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
