package org.firstinspires.ftc.teamcode.Supers.Auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Supers.Robot;

/**
 * Created by moham on 1/27/18.
 */
@Autonomous
public class EncoderTurnTest extends Robot {
    private long startTime;

    @Override
    public void init() {
        super.init();
        super.initAutonomous();
    }

    @Override
    public void loop() {
        if(resetPosition){
            startTime=System.currentTimeMillis();
            resetPosition=false;
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBack.setTargetPosition(-572);
            leftFront.setTargetPosition(-572);
            rightBack.setTargetPosition(572);
            rightFront.setTargetPosition(572);

            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftBack.setPower(.2);
            leftFront.setPower(.2);
            rightBack.setPower(.2);
            rightFront.setPower(.2);
        }
        telemetry.addData("leftBack", leftBack.getCurrentPosition());
        telemetry.addData("leftFront", leftFront.getCurrentPosition());
        telemetry.addData("rightBack", rightBack.getCurrentPosition());
        telemetry.addData("rightFront", rightFront.getCurrentPosition());
        Log.d("GraphTest", String.format("%d;%d;%d;%d;%d", System.currentTimeMillis(), leftBack.getCurrentPosition(), leftFront.getCurrentPosition(), rightBack.getCurrentPosition(), rightFront.getCurrentPosition()));

    }

    @Override
    public void stop() {
        super.stop();
    }
}
