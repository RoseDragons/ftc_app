package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name = "BluePark", group = "Opmode")
public class BluePark extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime();

    private DcMotor right_drive;
    private DcMotor left_drive;
    private DcMotor arm;

    private Servo grab_right;
    private Servo grab_left;
    private Servo intake;

    // The IMU (Internal Measurement Unit) sensor object
    BNO055IMU imu;

    // State used for updating telemetry for IMU sensor
    Orientation angles;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        right_drive = hardwareMap.dcMotor.get("right_drive");
        left_drive = hardwareMap.dcMotor.get("left_drive");
        arm = hardwareMap.dcMotor.get("arm");

        grab_right = hardwareMap.servo.get("grab_right");
        grab_left = hardwareMap.servo.get("grab_left");

        grab_left.setDirection(Servo.Direction.REVERSE);

        intake = hardwareMap.servo.get("intake");

        initImu();

        double intakePosition = 1;

        // Reverse one of the drive motors.
        left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        // Arm motor is also installed reverse
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {
            intake.setPosition(intakePosition);

            grab_left.setPosition(0.5);
            grab_right.setPosition(0.5);

            ////////////!!! Add autonomous code here !!!
            // pause(in milliseconds)

            // Go straight
            runStraight(900, 0.5);
            //pause(60000);


            pause(1000);

            //Turn left 90 degrees
            turnToAngle(0.5,-90);


            //Park under alliance bridge
            runStraight(1400,  0.5);

            telemetry.addData("Left Pow", left_drive.getPower());
            telemetry.addData("Right Pow", right_drive.getPower());
            telemetry.addData("Arm Pow", arm.getPower());
            telemetry.addData("Intake Pos", intakePosition);
            telemetry.addData("Grabber Pos", grab_left.getPosition());
            telemetry.update();
        }
    }

    private void initImu() {
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
    }

    protected boolean isMotorBusy(DcMotor dcMotor) {
        int targetPosition = dcMotor.getTargetPosition();
        int currentPosition = dcMotor.getCurrentPosition();

        if (targetPosition > 0 && currentPosition > targetPosition) {
            return false;
        }
        if (targetPosition < 0 && currentPosition < targetPosition) {
            return false;
        }
        return true;
    }

    protected void runStraight(int position, double pow) {

        // Reset encoders
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_drive.setTargetPosition(-position);
        right_drive.setTargetPosition(-position);

        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (position > 0) {
            left_drive.setPower(pow);
            right_drive.setPower(pow);
        } else {
            left_drive.setPower(-pow);
            right_drive.setPower(-pow);
        }

        while (opModeIsActive() && (isMotorBusy(left_drive) || isMotorBusy(right_drive))) {
            telemetry.addData("leftMotor", "%d, target: %d, power: %.2f", left_drive.getCurrentPosition(), left_drive.getTargetPosition(), left_drive.getPower());
            telemetry.addData("rightMotor", "%d, target: %d, power: %.2f", right_drive.getCurrentPosition(), right_drive.getTargetPosition(), right_drive.getPower());
            telemetry.update();
        }

        left_drive.setPower(0);
        right_drive.setPower(0);
    }

    protected void runToPositions(int leftPosition, double leftPower, int rightPosition, double rightPower) {

        // Reset encoders
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_drive.setTargetPosition(-leftPosition);
        right_drive.setTargetPosition(-rightPosition);

        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (leftPosition > 0) {
            left_drive.setPower(leftPower);
        } else {
            left_drive.setPower(-leftPower);
        }

        if (rightPosition > 0) {
            right_drive.setPower(rightPower);
        } else {
            right_drive.setPower(-rightPower);
        }

        while (opModeIsActive() && (isMotorBusy(left_drive) || isMotorBusy(right_drive))) {
            telemetry.addData("leftMotor", "%d, target: %d, power: %.2f", left_drive.getCurrentPosition(), left_drive.getTargetPosition(), left_drive.getPower());
            telemetry.addData("rightMotor", "%d, target: %d, power: %.2f", right_drive.getCurrentPosition(), right_drive.getTargetPosition(), right_drive.getPower());
            telemetry.update();
        }

        left_drive.setPower(0);
        right_drive.setPower(0);
    }

    /**
     * Turns to given angle using gyro
     * @param power
     * @param targetHeading
     */
    protected void turnToAngle(double power, float targetHeading) {
        float startHeading = imu.getAngularOrientation().firstAngle;
        float diff = targetHeading - startHeading;

        if (diff < 0) {
            // positive power means turn left, negative means turn right
            power = -power;
        }

        targetHeading = AngleUnit.normalizeDegrees(targetHeading);
        float currentHeading = startHeading;

        while(opModeIsActive() &&
                ((diff > 0 && currentHeading < targetHeading)
                        || (diff < 0 && currentHeading > targetHeading))) {

            left_drive.setPower(-power);
            right_drive.setPower(power);

            telemetry.clearAll();
            telemetry.addData("Start Heading", "%.2f", startHeading);
            telemetry.addData("Target Heading", "%.2f", targetHeading);
            telemetry.addData("Current Heading", "%.2f", currentHeading);
            telemetry.update();

            currentHeading = imu.getAngularOrientation().firstAngle;
        }

        stopAllDrive();
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

    // Stop moving robot base
    private void stopAllDrive() {
        left_drive.setPower(0);
        right_drive.setPower(0);
    }

    //All motor power is set to 0
    private void stopRobot() {
        stopAllDrive();

        arm.setPower(0);

        intake.setPosition(0.5);

        grab_left.setPosition(1);
        grab_right.setPosition(1);
    }

    public void pause(long ms) {
        try {
            stopAllDrive();
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}
