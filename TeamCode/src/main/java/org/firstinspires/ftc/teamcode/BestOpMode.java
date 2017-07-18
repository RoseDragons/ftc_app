package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Marjorie Blanco on 7/17/2017.
 */

public class BestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftMotor = hardwareMap.dcMotor.get("left_motor");
        DcMotor rightMotor = hardwareMap.dcMotor.get("right_motor");

        waitForStart();
        while (opModeIsActive()) {
            leftMotor.setPower(1);
            rightMotor.setPower(1);

            wait(1000);
            leftMotor.setPower(0);
            rightMotor.setPower(0);

        }
    }
}
