package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by Rose Dragons on 11/23/2017.
 */
public abstract class AutoDrive extends EmberBot {

    private int jewelKnockingPosition = 850;

    @Override
    public void runOpMode() {
        super.runOpMode();

        colorSensorServo.setPosition(0.001);
        while (distanceSensor.getDistance(DistanceUnit.CM) > 7.0);

        sleep(1000);

        if (colorSensor.red() > colorSensor.blue()) {
            if (getAlianceColor() == Color.RED) {
                knockBackward();
            } else {
                knockForward();
            }
        } else {
            if (getAlianceColor() == Color.BLUE) {
                knockBackward();
            } else {
                knockForward();
            }
        }

        colorSensorServo.setPosition(0.47);

        telemetry.addData("Alpha", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());

        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", distanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("Sensor Servo", "%.2f", colorSensorServo.getPosition());
        telemetry.update();
    }

    protected void runToPositions(int turningSidePosition, int oppositeSidePosition) {

        // Reset encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        getTurningSideMotor().setTargetPosition(turningSidePosition);
        getOppositeSideMotor().setTargetPosition(oppositeSidePosition);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double turningSidePower = 0.35;
        if (turningSidePosition < 0) {
            turningSidePower = -turningSidePower;
        }
        getTurningSideMotor().setPower(turningSidePower);

        double oppositeSidePower = 0.35;
        if (oppositeSidePosition < 0) {
            oppositeSidePower = -oppositeSidePower;
        }
        getOppositeSideMotor().setPower(oppositeSidePower);

        while (opModeIsActive() && (isMotorBusy(leftMotor) || isMotorBusy(rightMotor))) {
            telemetry.addData("leftMotor", "%d, target: %d, power: %.2f", leftMotor.getCurrentPosition(), leftMotor.getTargetPosition(), leftMotor.getPower());
            telemetry.addData("rightMotor", "%d, target: %d, power: %.2f", rightMotor.getCurrentPosition(), rightMotor.getTargetPosition(), rightMotor.getPower());
            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private void knockForward() {
        runToPositions(jewelKnockingPosition, jewelKnockingPosition);
        runToPositions(-jewelKnockingPosition, -jewelKnockingPosition);
    }

    protected void knockBackward(){
        runToPositions(-jewelKnockingPosition, -jewelKnockingPosition);
        runToPositions(jewelKnockingPosition, jewelKnockingPosition);
    }

    abstract public DcMotor getTurningSideMotor();
    abstract public DcMotor getOppositeSideMotor();
    abstract public int getAlianceColor();
}