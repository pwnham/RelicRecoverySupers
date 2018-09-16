package org.firstinspires.ftc.teamcode.Supers.Auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Supers.Robot;

/**
 * Created by moham on 3/31/18.
 */
@TeleOp
public class JoyStickTesting extends Robot {

    public void init() {
        super.init();
    }

    public void loop() {
        telemetry.addData("y", gamepad1.left_stick_y);
        telemetry.addData("x", gamepad1.left_stick_x);
    }
}
