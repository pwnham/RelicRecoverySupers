package org.firstinspires.ftc.teamcode.Supers.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Supers.Robot;

/**
 * Created by moham on 1/25/18.
 */
@Autonomous
public class StraightDriveNoEncoder extends Robot {

    private long startTime;

    @Override
    public void init() {
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        slideLeft = (DcMotorEx) hardwareMap.dcMotor.get("slideLeft");
        slideRight = (DcMotorEx) hardwareMap.dcMotor.get("slideRight");
        intakeLeft = (DcMotorEx) hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = (DcMotorEx) hardwareMap.dcMotor.get("intakeRight");

        flipLeft = hardwareMap.servo.get("flipLeft");
        flipRight = hardwareMap.servo.get("flipRight");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flipLeft.setPosition(flipLeftDown);
        flipRight.setPosition(flipRightDown);

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {
        leftBack.setPower(.3);
        leftFront.setPower(.3);
        rightBack.setPower(.3);
        rightFront.setPower(.3);
    }

    @Override
    public void stop() {
        super.stop();
    }
}
