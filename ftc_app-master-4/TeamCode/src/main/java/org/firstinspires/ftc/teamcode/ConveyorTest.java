package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by moham on 11/25/17.
 */

@Autonomous
public class ConveyorTest extends OpMode {
    Servo conveyorLeft, conveyorRight;
    public double conveyorLeftStart;
    public double conveyorRightStart;
    public double grabRightOut = .8;
    public double grabRightIn = .7;
    public double grabLeftOut = .05;
    public double grabLeftIn = .2;

    @Override
    public void init() {
        conveyorLeft = hardwareMap.servo.get("conveyorLeft");
        conveyorRight = hardwareMap.servo.get("conveyorRight");
        conveyorLeft.setPosition(grabLeftOut);
        conveyorRight.setPosition(grabRightOut);
        conveyorLeftStart = conveyorLeft.getPosition();
        conveyorRightStart = conveyorRight.getPosition();
    }

    @Override
    public void loop() {

//        //intake IN
//        if (gamepad2.right_stick_y < -.1) {
//            conveyorLeftStart -= .05;
//            conveyorRightStart += .05;
//        } //intake OUT
//        else if (gamepad2.right_stick_y > .1) {
//            conveyorLeftStart += .05;
//            conveyorRightStart -= .05;
//        } else { //BASE CASE
//            conveyorLeftStart = .50;
//            conveyorRightStart = .50;
//        }

//        conveyorLeft.setPosition(conveyorLeft.getPosition() - .2);
//        conveyorRight.setPosition(conveyorLeft.getPosition() + .2);
////        conveyorLeftStart += .1;
////        conveyorRightStart -= .1;
////        conveyorLeft.setPosition(conveyorLeftStart);
////        conveyorRight.setPosition(conveyorRightStart);
        conveyorLeft.setPosition(grabLeftIn);
        conveyorRight.setPosition(grabRightIn);
        telemetry.addData("Left", conveyorLeft.getPosition());
        telemetry.addData("Right", conveyorRight.getPosition());
    }
}
