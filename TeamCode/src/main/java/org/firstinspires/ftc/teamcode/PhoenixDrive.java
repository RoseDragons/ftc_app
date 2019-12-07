package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "PhoenixDrive", group = "")
public class PhoenixDrive extends LinearOpMode {

    private DcMotor right_drive;
    private DcMotor left_drive;
    private DcMotor arm;

    private Servo grab_right;
    private Servo grab_left;
    private Servo intake;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        right_drive = hardwareMap.dcMotor.get("right_drive");
        left_drive = hardwareMap.dcMotor.get("left_drive");
        arm = hardwareMap.dcMotor.get("arm");

        grab_right = hardwareMap.servo.get("grab_right");
        grab_left = hardwareMap.servo.get("grab_left");

        grab_left.setDirection(Servo.Direction.REVERSE);

        intake = hardwareMap.servo.get("intake");

        // Reverse one of the drive motors.
        left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        // Arm motor is also installed reverse
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                // The Y axis of a joystick ranges from -1 in its topmost position
                // to +1 in its bottommost position. We negate this value so that
                // the topmost position corresponds to maximum forward power.
                left_drive.setPower(-gamepad1.right_stick_y + gamepad1.right_stick_x);
                right_drive.setPower(-gamepad1.right_stick_y - gamepad1.right_stick_x);

                double armPower = -gamepad2.right_stick_y;
                if (!gamepad2.left_bumper) {
                    armPower /= 2;
                }
                arm.setPower(armPower);

                double intakePosition = (gamepad2.left_stick_y + 1) / 2;
                intake.setPosition(intakePosition);

                if (gamepad2.dpad_up){
                    grab_left.setPosition(1);
                    grab_right.setPosition(1);
                } else {
                    grab_left.setPosition(0.5);
                    grab_right.setPosition(0.5);
                }

                telemetry.addData("leftMotor", "%d, power: %.2f", left_drive.getCurrentPosition(), left_drive.getPower());
                telemetry.addData("rightMotor", "%d, power: %.2f", right_drive.getCurrentPosition(), right_drive.getPower());
                telemetry.addData("Arm Pow", arm.getPower());
                telemetry.addData("Intake Pos", intakePosition);
                telemetry.addData("Grabber Pos", grab_left.getPosition());
                telemetry.update();
            }
        }
    }
}