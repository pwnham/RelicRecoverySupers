package org.firstinspires.ftc.teamcode.Supers.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Created by moham on 2/23/18.
 */
@TeleOp
public class RelicSlideSolo extends OpMode{

    public CRServo relic1, relic2, relic3, relic4;

    @Override
    public void init() {
        relic1 = hardwareMap.crservo.get("relic1");
        relic2 = hardwareMap.crservo.get("relic2");
        relic3 = hardwareMap.crservo.get("relic3");
        relic4 = hardwareMap.crservo.get("relic4");
    }

    @Override
    public void loop() {

    //relic slide
        if (gamepad2.left_stick_y < -.5) {
            relic1.setPower(.75);
            relic2.setPower(.75);
            relic3.setPower(.75);
            relic4.setPower(.75);
        } else if (gamepad2.left_stick_y > .5) {
            relic1.setPower(-.75);
            relic2.setPower(-.75);
            relic3.setPower(-.75);
            relic4.setPower(-.75);
        } else {
            relic1.setPower(0);
            relic2.setPower(0);
            relic3.setPower(0);
            relic4.setPower(0);
        }

    }
}
