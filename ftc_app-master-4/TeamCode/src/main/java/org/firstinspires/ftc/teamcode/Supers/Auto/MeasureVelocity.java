package org.firstinspires.ftc.teamcode.Supers.Auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Supers.Robot;

import java.util.ArrayList;

/**
 * Created by moham on 3/31/18.
 */
@TeleOp
public class MeasureVelocity extends Robot {

    @Override
    public void init() {
        super.init();
    }

    private long startTime;
    private ArrayList<Long> timeStamp = new ArrayList<Long>();
    private ArrayList<Double> leftBackPos = new ArrayList<Double>();
    private ArrayList<Double> leftFrontPos = new ArrayList<Double>();
    private ArrayList<Double> rightBackPos = new ArrayList<Double>();
    private ArrayList<Double> rightFrontPos = new ArrayList<Double>();

    private double leftBackSum = 0, leftFrontSum = 0, rightBackSum = 0, rightFrontSum = 0;

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

            telemetry.addData("leftBack", leftBack.getCurrentPosition());
            telemetry.addData("leftFront", leftFront.getCurrentPosition());
            telemetry.addData("rightBack", rightBack.getCurrentPosition());
            telemetry.addData("rightFront", rightFront.getCurrentPosition());
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
