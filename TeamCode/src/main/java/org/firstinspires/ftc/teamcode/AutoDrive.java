package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Rose Dragons on 11/23/2017.
 */
public abstract class AutoDrive extends EmberBot {

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
    }

    abstract public DcMotor getTurningSideMotor();
    abstract public DcMotor getOppositeSideMotor();
}