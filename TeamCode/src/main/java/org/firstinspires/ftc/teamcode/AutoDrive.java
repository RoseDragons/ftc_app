package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Rose Dragons on 11/23/2017.
 */
public abstract class AutoDrive extends EmberBot {

    @Override
    public void runOpMode() {
        super.runOpMode();

        // Reset encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        getTurningSideMotor().setTargetPosition(6500);
        getOppositeSideMotor().setTargetPosition(8000);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        getTurningSideMotor().setPower(0.8);
        getOppositeSideMotor().setPower(0.8);

        while (opModeIsActive() &&
                (leftMotor.isBusy() && rightMotor.isBusy())) {

            telemetry.addData("Path1", "leftMotor: %d", leftMotor.getCurrentPosition());
            telemetry.addData("Path2", "rightMotor: %d", rightMotor.getCurrentPosition());
            telemetry.update();
        }

        while (opModeIsActive()) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }

    abstract public DcMotor getTurningSideMotor();
    abstract public DcMotor getOppositeSideMotor();
}