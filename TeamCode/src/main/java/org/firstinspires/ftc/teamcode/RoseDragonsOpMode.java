package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Rose Dragons on 9/24/2017.
 */
@TeleOp(name="Rose Dragons OpMode", group="Linear Opmode")
public class RoseDragonsOpMode extends EmberBot {
    @Override
    public void runOpMode() {
        super.runOpMode();

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
            rightMotor.setPower(rightPower);
            armMotor1.setPower(armPower);
            armMotor2.setPower(armPower);
            leftServo.setPosition(leftServoPosition);
            rightServo.setPosition(rightServoPosition);

            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("ArnMotor", "left (%.2f)", armPower);
            telemetry.addData("Servo Motors", "left (%.2f), right (%.2f)", leftServoPosition, rightServoPosition);
            telemetry.update();
        }
    }
}