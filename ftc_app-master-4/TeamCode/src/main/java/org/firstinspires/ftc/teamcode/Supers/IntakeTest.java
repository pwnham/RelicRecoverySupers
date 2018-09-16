package org.firstinspires.ftc.teamcode.Supers;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

/**
 * Created by moham on 3/15/18.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class IntakeTest extends Robot {

    long startTime;
    int intakeState = 0;

    private long startTimeIntake;


    private ArrayList<Long> timeStamp = new ArrayList<Long>();
    private ArrayList<Double> intakeLeftPos = new ArrayList<Double>();
    private ArrayList<Double> intakeRightPos = new ArrayList<Double>();

    public void init() {
        super.init();
    }

    public void loop() {
        if(!armLifted) {
            jewelArm.setPosition(jewelArmUp);
            relicFlipper.setPosition(relicFlipperUp);
            armLifted = true;
        }

        double[] intakeVelocities = getVelocity(intakeLeft, intakeRight);
        telemetry.addData("Intake Left Velocity", intakeVelocities[0]);
        telemetry.addData("Intake Right Velocity", intakeVelocities[1]);
//        switch (intakeState) {
//            case 0:
//                if(resetPosition){
//                    startTime=System.currentTimeMillis();
//                    resetPosition=false;
//                }
//                if (System.currentTimeMillis() - startTime > 1000) {
//                    if (intakeLeft.getVelocity(AngleUnit.DEGREES) < 1200 || intakeRight.getVelocity(AngleUnit.DEGREES) < 1200) {
//                        intakeState = 1;
//                        resetPosition = true;
//                    }
//                } else {
//                    break;
//                }
//                intakeLeft.setPower(1);
//                intakeRight.setPower(1);
//                break;
//            case 1:
//                if(resetPosition){
//                    startTime=System.currentTimeMillis();
//                    resetPosition=false;
//                }
//                if (System.currentTimeMillis() - startTime > 500) {
//                    resetPosition = true;
//                    intakeState = 0;
//                } else {
//                    intakeLeft.setPower(-1);
//                    intakeRight.setPower(1);
//                }
//        }

//        intakeLeft.setPower(1);
//        intakeRight.setPower(1);
//
//        telemetry.addData("intakeLeft", intakeLeft.getVelocity(AngleUnit.DEGREES));
//        telemetry.addData("intakeRight", intakeRight.getVelocity(AngleUnit.DEGREES));
//        telemetry.addData("intakeState", intakeState);
//
//        rightBack.setPower(((0.91614053*.2 - 0.04007548)));
//        rightFront.setPower(((0.91606515*.2 - 0.03991444)));
//        leftBack.setPower(((0.91720407*.2 - 0.03995359)));
//        leftFront.setPower(((0.91352592*.2 - 0.03907521)));

        if (resetPosition) {
            startTime = System.currentTimeMillis();
            resetPosition = false;
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBack.setTargetPosition(1450);
            leftFront.setTargetPosition(1450);
            rightBack.setTargetPosition(1450);
            rightFront.setTargetPosition(1450);

            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftBack.setPower(.3);
            leftFront.setPower(.3);
            rightBack.setPower(.3);
            rightFront.setPower(.3);
        }

        switch (intakeState) {
            case 0:
                if (resetIntake) {
                    startTimeIntake = System.currentTimeMillis();
                    resetIntake = false;
                }
                if (System.currentTimeMillis() - startTimeIntake > 750) {
                    if (intakeVelocities[0] < 700 || intakeVelocities[1] < 700) {
                        intakeState = 1;
                        resetIntake = true;
                    }
                }
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
                break;
            case 1:
                if (resetIntake) {
                    startTimeIntake = System.currentTimeMillis();
                    resetIntake = false;
                }
                if (System.currentTimeMillis() - startTimeIntake > 250) {
                    intakeState = 0;
                    resetIntake = true;
                } else {
                    intakeLeft.setPower(-1);
                    intakeRight.setPower(-1);
                }
                break;

        }

//        intakeLeftPos.add(intakeVelocities[0]);
//        intakeRightPos.add(intakeVelocities[1]);
        intakeLeftPos.add((double)intakeLeft.getCurrentPosition());
        intakeRightPos.add((double)intakeRight.getCurrentPosition());
        timeStamp.add(System.currentTimeMillis());

    }

    @Override
    public void stop() {

        Log.d("timeStamp", timeStamp.toString());

        Log.d("intakeLeftPos", intakeLeftPos.toString());
        Log.d("intakeRightPos", intakeRightPos.toString());

    }
}
