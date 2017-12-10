package org.firstinspires.ftc.teamcode;

/**
 * Created by Rose Dragons on 12/9/2017.
 */
abstract public class BackAutoDrive extends AutoDrive {
    @Override
    public void runOpMode() {
        super.runOpMode();

        moveArmTicks(2500);

        telemetry.addLine("arm motion1 done");
        telemetry.update();

        runToPositions(6400, 6400);
        telemetry.addLine("straight done");
        telemetry.update();

        runToPositions(-900, 1000);
        runToPositions(300, 300);
        runToPositions(-200, -200);
        telemetry.addLine("all done");
        telemetry.update();

        while (opModeIsActive()) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }
}
