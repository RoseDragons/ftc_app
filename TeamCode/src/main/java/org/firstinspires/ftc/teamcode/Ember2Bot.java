package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Ember2Bot extends LinearOpMode {
    protected static final double CONTINUOUS_SERVO_STOP = 0.5;
    protected static final double CONTINUOUS_SERVO_FORWARD = 1.0;
    protected static final double CONTINUOUS_SERVO_REVERSE = 0.0;

    protected static final int ACC_MOTOR_MAX_TICKS = 28000;

    // Declare OpMode members.
    protected ElapsedTime runtime = new ElapsedTime();

    DcMotor Motor_0;
    DcMotor Motor_1;
    DcMotor Motor_2;
    DcMotor Motor_3;

    DcMotor AccMotor;
    DigitalChannel AccTouch;

    //CRServo servo_5;
    //CRServo servo_3;

    double v0;
    double v1;
    double v2;
    double v3;

    @Override
    public void runOpMode() {
        emberInit();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        emberStart();
        while (opModeIsActive()) {
            emberLoop();
        }
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

        Motor_0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motor_0.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_1.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_2.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_3.setDirection(DcMotorSimple.Direction.REVERSE);

        AccMotor = hardwareMap.get(DcMotor.class, "torqueNADO");
        AccMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AccMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        AccTouch = hardwareMap.get(DigitalChannel.class, "AccButton");
        AccTouch.setMode(DigitalChannel.Mode.INPUT);

        //servo_3 = hardwareMap.get(CRServo.class, "Servo_3");
        //servo_5 = hardwareMap.get(CRServo.class, "Servo_5");
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void emberStart() {
        runtime.reset();

        telemetry.addData("Status", "Initialized");
    }

    /**
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void emberLoop() {};

    /**
     * This method will be called when this op mode is first disabled
     */
    public void emberStop() {
        Motor_0.setPower(0);
        Motor_1.setPower(0);
        Motor_2.setPower(0);
        Motor_3.setPower(0);

        AccMotor.setPower(0);

        //servo_3.setPower(0);
        //servo_5.setPower(0);
    }


    protected void initAcc() {
        // Move acctuator until button is press
        AccMotor.setPower(-1);

        while (AccTouch.getState() == true) {
            telemetry.addData("accMotor", "pos: %d", AccMotor.getCurrentPosition());
            telemetry.update();
        }

        AccMotor.setPower(0);

        AccMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        AccMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void moveAccTicks(int ticks, double AccPower) {
        AccMotor.setTargetPosition(ticks + AccMotor.getCurrentPosition());

        if (ticks < 0) {
            AccPower = -AccPower;
        }
        AccMotor.setPower(AccPower);

        while (opModeIsActive() &&
                isMotorBusy(AccMotor, ticks > 0) &&
                inAccRange(ACC_MOTOR_MAX_TICKS, ticks > 0)) {
            //Motor has power and is running during this loop

            telemetry.addData("accMotor", "pos: %d", AccMotor.getCurrentPosition());
            telemetry.addData("accMotor", "tartger pos: %d", AccMotor.getTargetPosition());
            telemetry.update();
        }

        AccMotor.setPower(0.0);
    }

    protected boolean isMotorBusy(DcMotor dcMotor, boolean isForward) {
        int targetPosition = dcMotor.getTargetPosition();
        int currentPosition = dcMotor.getCurrentPosition();

        if (isForward && currentPosition > targetPosition) {
            return false;
        }
        if (!isForward && currentPosition < targetPosition) {
            return false;
        }
        return true;
    }

    protected boolean inAccRange(int max, boolean isForward) {

        if (isForward) {
            return (AccMotor.getCurrentPosition() < max);

        } else {
            if (AccTouch.getState() == false) {

                // If button is pressed, we cannot go further down
                AccMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                AccMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                return false;
            }

            return true;
        }
    }
}
