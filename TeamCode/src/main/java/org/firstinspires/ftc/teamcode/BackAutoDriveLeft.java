package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Rose Dragons on 9/24/2017.
 */
@Autonomous(name="Back Auto Drive Left", group="Pushbot")
public class BackAutoDriveLeft extends AutoDrive {
    @Override
    public void runOpMode() {
        super.runOpMode();

        moveArmTicks(2500);

        telemetry.addLine("arm motion1 done");
        telemetry.update();

        runToPositions(6400, 6400);
        telemetry.addLine("straight done");
        telemetry.update();

        runToPositions(-500, 750);
        runToPositions(500, 500);
        runToPositions(-200, -200);
        telemetry.addLine("all done");
        telemetry.update();

        while (opModeIsActive()) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }

    @Override
    public DcMotor getTurningSideMotor() {
        return super.leftMotor;
    }

    @Override
    public DcMotor getOppositeSideMotor() {
        return super.rightMotor;
    }
}