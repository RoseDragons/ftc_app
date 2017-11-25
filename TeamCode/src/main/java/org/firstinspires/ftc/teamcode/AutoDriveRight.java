package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Rose Dragons on 9/29/2017.
 */
@Autonomous(name="Auto Drive Right ", group="Pushbot")
public class AutoDriveRight extends AutoDrive {

    @Override
    public DcMotor getTurningSideMotor() {
        return super.rightMotor;
    }

    @Override
    public DcMotor getOppositeSideMotor() {
        return super.leftMotor;
    }
}
