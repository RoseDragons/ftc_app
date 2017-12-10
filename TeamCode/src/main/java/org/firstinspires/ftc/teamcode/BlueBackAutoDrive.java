package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Rose Dragons on 9/24/2017.
 */
@Autonomous(name="Blue Auto Drive Back", group="Pushbot")
public class BlueBackAutoDrive extends BackAutoDrive {
    @Override
    public DcMotor getTurningSideMotor() {
        return super.rightMotor;
    }

    @Override
    public DcMotor getOppositeSideMotor() {
        return super.leftMotor;
    }

    @Override
    public int getAlianceColor() { return Color.BLUE; }
}