package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Locale;

public abstract class NemoBot extends LinearOpMode {
    // The IMU (Internal Measurement Unit) sensor object
    BNO055IMU imu;

    // State used for updating telemetry for IMU sensor
    Orientation angles;

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

    /**
     * NOTE: NOT WORKING - Do not use
     * @param power
     * @param distance_cm
     */
    protected void driveDistance(double power, double distance_cm) {
        imu.startAccelerationIntegration(null, null, 250);

        if (distance_cm > 0) {
            power = -power;
        }

        mecanumDrive(0, 0, 0, power);

        while(opModeIsActive()) {
            Position position = imu.getPosition();
            position = position.toUnit(DistanceUnit.CM);
            if ((distance_cm > 0 && position.x > distance_cm) ||
                    (distance_cm < 0  && position.x < distance_cm)) {
                break;
            }

            telemetry.clearAll();
            telemetry.addData("position", position.toString());
            telemetry.update();
        }

        stopAllDrive();
        imu.stopAccelerationIntegration();
    }

    protected void turnDegrees(double power, float degrees) {
        Orientation orientation  = imu.getAngularOrientation();

        turnToAngle(power, orientation.firstAngle + degrees);
    }

    /**
     * Turns to given angle using gyro
     * @param power
     * @param targetHeading
     */
    protected void turnToAngle(double power, float targetHeading) {
        float startHeading = imu.getAngularOrientation().firstAngle;
        float diff = targetHeading - startHeading;


        if (diff > 0) {
            // positive power means turn left, negative means turn right
            power = -power;
        }

        targetHeading = AngleUnit.normalizeDegrees(targetHeading);
        float currentHeading = startHeading;

        while(opModeIsActive() &&
                ((diff > 0 && currentHeading < targetHeading)
                        || (diff < 0 && currentHeading > targetHeading))) {

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
            angles    = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
                .addData("position", new Func<String>() {
                    @Override public String value() {
                        return position.toString();
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
