package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Rose Dragons on 9/24/2017.
 */
@Autonomous(name="Red Auto Drive Back", group="Pushbot")
@Disabled
public class RedBackAutoDrive extends BackAutoDrive {
    @Override
    public DcMotor getTurningSideMotor() {
        return super.leftMotor;
    }

    @Override
    public DcMotor getOppositeSideMotor() {
        return super.rightMotor;
    }

    @Override
    public int getAlianceColor() { return Color.RED; }
}