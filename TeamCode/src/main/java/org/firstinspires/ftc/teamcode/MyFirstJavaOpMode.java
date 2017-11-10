package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Rose Dragons on 9/24/2017.
 */
@TeleOp(name="First:Java OpMode", group="Linear Opmode")
public class MyFirstJavaOpMode extends LinearOpMode {
    //private Gyroscope imu;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor armMotor1;
    private DcMotor armMotor2;
    private Servo leftServo;
    private Servo rightServo;
    private ElapsedTime runtime = new ElapsedTime();

    //private DigitalChannel digitalTouch;
    //private DistanceSensor sensorColorRange;

    @Override
    public void runOpMode() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double armPower;
        double rightServoPosition = 0.0;
        double leftServoPosition = 0.0;

        telemetry.addData("Status", "starting");
        telemetry.update();

        //imu = hardwareMap.get(Gyroscope.class, "imu");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor1 = hardwareMap.get(DcMotor.class, "armMotor1");
        armMotor2 = hardwareMap.get(DcMotor.class, "armMotor2");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "ready to start");
        telemetry.update();

        leftServo.setPosition(1);
        rightServo.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y * 0.9;
            double turn  =  gamepad1.right_stick_x * 0.5;
            double up = gamepad2.left_stick_y * 0.8;
            double open = gamepad2.right_stick_y * 0.5;

            leftPower    = Range.clip(drive + turn, -0.9, 0.9) ;
            rightPower   = Range.clip(drive - turn, -0.9, 0.9) ;
            armPower = Range.clip(up, -0.8, 0.8);
            leftServoPosition = Range.clip(0.5 + open, 0, 1);
            rightServoPosition = Range.clip(0.5 - open, 0, 1);

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftMotor.setPower(leftPower);
            rightMotor.setPower(-rightPower);
            armMotor1.setPower(-armPower);
            armMotor1.setPower(-armPower);
            leftServo.setPosition(leftServoPosition);
            rightServo.setPosition(rightServoPosition);

            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("ArnMotor", "left (%.2f)", armPower);
            telemetry.addData("Servo Motors", "left (%.2f), right (%.2f)", leftServoPosition, rightServoPosition);
            telemetry.update();
        }
    }
}