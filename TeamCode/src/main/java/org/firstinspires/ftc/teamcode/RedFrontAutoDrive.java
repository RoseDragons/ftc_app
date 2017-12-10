package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Rose Dragons on 9/29/2017.
 */
@Autonomous(name="Red Auto Drive Front", group="Pushbot")
public class RedFrontAutoDrive extends FrontAutoDrive {

    @Override
    public DcMotor getTurningSideMotor() {
        return super.rightMotor;
    }

    @Override
    public DcMotor getOppositeSideMotor() {
        return super.leftMotor;
    }

    @Override
    public int getAlianceColor() { return Color.RED; }
}
