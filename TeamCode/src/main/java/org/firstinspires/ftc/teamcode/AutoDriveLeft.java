package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Rose Dragons on 9/24/2017.
 */
@Autonomous(name="Auto Drive Left", group="Pushbot")
public class AutoDriveLeft extends AutoDrive {
    @Override
    public DcMotor getTurningSideMotor() {
        return super.leftMotor;
    }

    @Override
    public DcMotor getOppositeSideMotor() {
        return super.rightMotor;
    }
}