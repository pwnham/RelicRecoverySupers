package org.firstinspires.ftc.teamcode.Supers.Auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Supers.Robot;

import java.util.ArrayList;

/**
 * Created by moham on 1/25/18.
 */
@Autonomous
public class StraightDriveNoEncoder extends Robot {

    private long startTime;

    private ArrayList<Long> timeStamp = new ArrayList<Long>();
    private ArrayList<Double> leftBackPos = new ArrayList<Double>();
    private ArrayList<Double> leftFrontPos = new ArrayList<Double>();
    private ArrayList<Double> rightBackPos = new ArrayList<Double>();
    private ArrayList<Double> rightFrontPos = new ArrayList<Double>();

    private double leftBackSum = 0, leftFrontSum = 0, rightBackSum = 0, rightFrontSum = 0;

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

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        if (resetPosition) {
            startTime = System.currentTimeMillis();
            resetPosition = false;
        }

//        leftFront.setVelocity(1250, AngleUnit.DEGREES);
//        leftBack.setVelocity(1250, AngleUnit.DEGREES);
//        rightFront.setVelocity(1250, AngleUnit.DEGREES);
//        rightBack.setVelocity(1250, AngleUnit.DEGREES);

        leftFront.setPower(.9);
        leftBack.setPower(.9);
        rightFront.setPower(.9);
        rightBack.setPower(.9);

        if ((System.currentTimeMillis() - startTime) > 1200) {
            timeStamp.add(System.currentTimeMillis());
//            leftBackPos.add(leftBack.getCurrentPosition());
//            leftFrontPos.add(leftFront.getCurrentPosition());
//            rightBackPos.add(rightBack.getCurrentPosition());
//            rightFrontPos.add(rightFront.getCurrentPosition());

            leftBackPos.add(leftBack.getVelocity(AngleUnit.DEGREES));
            leftFrontPos.add(leftFront.getVelocity(AngleUnit.DEGREES));
            rightBackPos.add(rightBack.getVelocity(AngleUnit.DEGREES));
            rightFrontPos.add(rightFront.getVelocity(AngleUnit.DEGREES));

//        Log.d("GraphTest", String.format("%d;%d;%d;%d;%d", System.currentTimeMillis(), leftBack.getCurrentPosition(), leftFront.getCurrentPosition(), rightBack.getCurrentPosition(), rightFront.getCurrentPosition()));

        }
    }


    @Override
    public void stop() {
        for (int i= 0; i<leftBackPos.size(); i++) {
            leftBackSum += leftBackPos.get(i);
            leftFrontSum += leftFrontPos.get(i);
            rightBackSum += rightBackPos.get(i);
            rightFrontSum += rightFrontPos.get(i);
        }

        Log.d("timeStamp", timeStamp.toString());

        Log.d("leftBackPos", leftBackPos.toString());
        Log.d("VelocityAverage", "leftBack " + leftBackSum/leftBackPos.size());

        Log.d("leftFrontPos", leftFrontPos.toString());
        Log.d("VelocityAverage", "leftFront " + leftFrontSum/leftFrontPos.size());

        Log.d("rightBackPos", rightBackPos.toString());
        Log.d("VelocityAverage", "rightBack " + rightBackSum/rightBackPos.size());

        Log.d("rightFrontPos", rightFrontPos.toString());
        Log.d("VelocityAverage", "rightFront " + rightFrontSum/rightFrontPos.size());
    }
}
