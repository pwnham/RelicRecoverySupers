package org.firstinspires.ftc.teamcode.Supers.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by moham on 2/23/18.
 */
@TeleOp
public class JewelSolo extends OpMode {

    private Servo jewelArm, jewelTurn;

    public double jewelArmUp = .275;
    public double jewelArmIn = .01;
    public double jewelArmDown = .95;
    public double jewelTurnCenter = .24;
    public double jewelTurnFront = .05;
    public double jewelTurnBack = .45;

    @Override
    public void init() {
        jewelArm = hardwareMap.servo.get("jewelArm");
        jewelTurn = hardwareMap.servo.get("jewelTurn");

        jewelArm.setPosition(jewelArmIn);
        jewelTurn.setPosition(jewelTurnCenter);
    }

    @Override
    public void loop() {

        //jewel arm
        if (gamepad1.dpad_up) {
            jewelArm.setPosition(jewelArmUp);
        } else if (gamepad1.dpad_down) {
            jewelArm.setPosition(jewelArmDown);
        } else if (gamepad1.dpad_left) {
            jewelArm.setPosition(jewelArmIn);
        }

        //jewel Turn
        if (gamepad1.a) {
            jewelTurn.setPosition(jewelTurnCenter);
        } if (gamepad1.x) {
            jewelTurn.setPosition(jewelTurnBack);
        } if (gamepad1.b) {
            jewelTurn.setPosition(jewelTurnFront);
        }

    }
}
