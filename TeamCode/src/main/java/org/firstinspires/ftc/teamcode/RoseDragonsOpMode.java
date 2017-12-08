package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Rose Dragons on 9/24/2017.
 */
@TeleOp(name="Rose Dragons OpMode", group="Linear Opmode")
public class RoseDragonsOpMode extends EmberBot {

    private double driveNormalFactor = 0.85;
    private double driveTurboFactor = 1.0;
    private double driveSlowFactor = 0.50;

    private double turnNormalFactor = 0.50;
    private double turnTurboFactor = 0.90;
    private double turnSlowFactor = 0.20;

    private double armNormalFactor = 0.50;
    private double armTurboFactor = 0.90;
    private double armSlowFactor = 0.15;


    @Override
    public void runOpMode() {
        super.runOpMode();

        moveArmTicks(-1350, 0.3);

        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double driveFactor = driveNormalFactor;
            double turnFactor = turnNormalFactor;
            if (gamepad1.right_bumper) {
                driveFactor = driveTurboFactor;
                turnFactor = turnTurboFactor;

            } else if (gamepad1.right_trigger > 0) {
                driveFactor = driveSlowFactor;
                turnFactor = turnSlowFactor;
            }

            double armFactor = armNormalFactor;
            if (gamepad2.right_bumper) {
                armFactor = armTurboFactor;
            } else if (gamepad2.right_trigger > 0) {
                armFactor = armSlowFactor;
            }

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y * driveFactor;
            double turn  =  gamepad1.right_stick_x * turnFactor;
            double up = gamepad2.left_stick_y * armFactor;
            double open = gamepad2.right_stick_y * 0.5;

            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            armPower = Range.clip(up, -1.0, 1.0);
            leftServoPosition = Range.clip(0.5 + open, 0.35, 1);
            // right servo is put 5 degree offV
            rightServoPosition = Range.clip(0.40 - open, 0, 0.65);

            //if (gamepad2.right_bumper == false) {
               // if ((armMotor2.getCurrentPosition() <= 0 && armPower < 0) || (armMotor2.getCurrentPosition() >= 4500 && armPower > 0)) {
                    //armPower = 0;
                //}
            //}

            // Send calculated power to wheels
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);
            armMotor1.setPower(armPower);
            armMotor2.setPower(armPower);
            leftServo.setPosition(leftServoPosition);
            rightServo.setPosition(rightServoPosition);

            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("ArmMotor", "position: %d, power: %.2f", armMotor2.getCurrentPosition(), armMotor2.getPower());
            telemetry.addData("Servo Motors", "left (%.2f), right (%.2f)", leftServoPosition, rightServoPosition);
            telemetry.update();
        }
    }
}