package org.firstinspires.ftc.teamcode.Supers.Auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Supers.Robot;

/**
 * Created by moham on 3/31/18.
 */
@TeleOp
public class SlideMeasure extends Robot {

    public void init() {
        super.init();
    }

    public void loop() {
        if(!armLifted) {
            jewelArm.setPosition(jewelArmUp);
            armLifted = true;
        }

        //lower slide
        if (gamepad2.x) {
            slideLeft.setPower(.65);
            slideRight.setPower(.65);
        } else if (gamepad2.y) { // raise slide
            slideLeft.setPower(-.99);
            slideRight.setPower(-.99);
        } else {
            slideLeft.setPower(0);
            slideRight.setPower(0);
        }


        telemetry.addData("left ticks", slideLeft.getCurrentPosition());
        telemetry.addData("right ticks", slideRight.getCurrentPosition());
    }
}
