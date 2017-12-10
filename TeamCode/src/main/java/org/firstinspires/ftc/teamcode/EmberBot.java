package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Rose Dragons on 11/24/2017.
 */

public abstract class EmberBot extends LinearOpMode {
    //protected Gyroscope imu;
    protected DigitalChannel digitalTouch;
    protected ColorSensor colorSensor;
    protected DistanceSensor distanceSensor;
    protected DcMotor leftMotor;
    protected DcMotor rightMotor;
    protected DcMotor armMotor1;
    protected DcMotor armMotor2;
    protected Servo leftServo;
    protected Servo rightServo;
    protected Servo colorSensorServo;
    protected ElapsedTime runtime = new ElapsedTime();


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
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensorColorRange");

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");

        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor1 = hardwareMap.get(DcMotor.class, "armMotor1");
        armMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor2 = hardwareMap.get(DcMotor.class, "armMotor2");
        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        colorSensorServo = hardwareMap.get(Servo.class, "colorSensorServo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "ready to start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
    }

    protected void moveArmTicks(int ticks) {
        moveArmTicks(ticks, 0.5);
    }

    protected void moveArmTicks(int ticks, double armPower) {
        // Reset encoders
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setTargetPosition(ticks);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (ticks < 0) {
            armPower = -armPower;
        }
        armMotor2.setPower(armPower);
        armMotor1.setPower(armPower);

        while (opModeIsActive() && isMotorBusy(armMotor2)) {
            telemetry.addData("armMotor2", "%d", armMotor2.getCurrentPosition());
            telemetry.update();
        }

        armMotor1.setPower(0.0);
        armMotor2.setPower(0.0);
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

}
