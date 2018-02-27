package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by moham on 12/1/17.
 */
@TeleOp
public class DriveTrainTest extends OpMode{
    DcMotor rightBack, rightFront, leftBack, leftFront;

    @Override
    public void init() {
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        rightBack.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        leftFront.setPower(0);

    }

    @Override
    public void loop() {
        rightBack.setPower(gamepad1.right_stick_y);
        rightFront.setPower(gamepad1.right_stick_y);
        leftBack.setPower(gamepad1.left_stick_y);
        leftFront.setPower(gamepad1.left_stick_y);

        telemetry.addData("rightStick y", gamepad1.right_stick_y);
        telemetry.addData("leftStick y", gamepad1.left_stick_y);
    }
}
