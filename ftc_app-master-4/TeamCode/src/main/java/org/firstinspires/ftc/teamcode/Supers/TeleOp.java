package org.firstinspires.ftc.teamcode.Supers;

/**
 * Created by moham on 1/15/18.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends Robot{

    private double drivePower, driveTurn;

    @Override
    public void init(){
        super.init();
    }

    @Override
    public void loop(){
        if(!armLifted) {
            jewelArm.setPosition(jewelArmUp);
            armLifted = true;
        }

        drivePower = gamepad1.left_stick_y;
        driveTurn = gamepad1.right_stick_x;

        if (gamepad1.left_bumper) {
            rightBack.setPower(-(drivePower + driveTurn) * .3);
            rightFront.setPower(-(drivePower + driveTurn) * .3);
            leftBack.setPower(-(drivePower - driveTurn) * .3);
            leftFront.setPower(-(drivePower - driveTurn) * .3);
        } else {
            rightBack.setPower(-(drivePower + driveTurn));
            rightFront.setPower(-(drivePower + driveTurn));
            leftBack.setPower(-(drivePower - driveTurn));
            leftFront.setPower(-(drivePower - driveTurn));
        }

        //intake in
        if (gamepad1.a) {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
        } else if (gamepad1.x) {//intake out
            intakeLeft.setPower(-1);
            intakeRight.setPower(-1);
        } else {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }

        //raise slide
        if (gamepad1.b) {
            slideLeft.setPower(.65);
            slideRight.setPower(.65);
        } else if (gamepad1.y) { // lower slide
            slideLeft.setPower(-.65);
            slideRight.setPower(-.65);
        } else {
            slideLeft.setPower(0);
            slideRight.setPower(0);
        }

        //flip tray
        if (gamepad1.dpad_up) {
            flipLeft.setPosition(flipLeftUp);
            flipRight.setPosition(flipRightUp);
        } else if (gamepad1.dpad_down) {//lower tray
            flipLeft.setPosition(flipLeftDown);
            flipRight.setPosition(flipRightDown);
        } else if (gamepad1.dpad_left) {
            flipLeft.setPosition(flipLeftMiddle);
            flipRight.setPosition(flipRightMiddle);
        }
        telemetry.addData("flipRight", flipRight.getPosition());
        telemetry.addData("flipLeft", flipLeft.getPosition());

    }

}
