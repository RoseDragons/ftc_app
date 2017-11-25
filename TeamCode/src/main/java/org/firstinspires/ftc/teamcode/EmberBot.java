package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Rose Dragons on 11/24/2017.
 */

public abstract class EmberBot extends LinearOpMode {
    //protected Gyroscope imu;
    protected DcMotor leftMotor;
    protected DcMotor rightMotor;
    protected DcMotor armMotor1;
    protected DcMotor armMotor2;
    protected Servo leftServo;
    protected Servo rightServo;
    protected ElapsedTime runtime = new ElapsedTime();

    //protected DigitalChannel digitalTouch;
    //protected DistanceSensor sensorColorRange;

    // Setup a variable for each drive wheel to save power level for telemetry
    protected double leftPower;
    protected double rightPower;
    protected double armPower;
    protected double rightServoPosition = 0.0;
    protected double leftServoPosition = 0.0;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "starting");
        telemetry.update();

        //imu = hardwareMap.get(Gyroscope.class, "imu");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");

        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor1 = hardwareMap.get(DcMotor.class, "armMotor1");
        armMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor2 = hardwareMap.get(DcMotor.class, "armMotor2");
        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

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
    }
}
