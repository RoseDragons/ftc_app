package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Rose Dragons on 9/29/2017.
 */
@Autonomous(name="Blue Auto Drive Front", group="Pushbot")
public class BlueFrontAutoDrive extends FrontAutoDrive {
    @Override
    public DcMotor getTurningSideMotor() {
        return super.leftMotor;
    }

    @Override
    public DcMotor getOppositeSideMotor() {
        return super.rightMotor;
    }

    @Override
    public int getAlianceColor() { return Color.BLUE; }
}
