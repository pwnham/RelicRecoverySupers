package org.firstinspires.ftc.teamcode.Supers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by moham on 1/26/18.
 */
@TeleOp
public class TeleOpTwoPlayer extends Robot {

    private double drivePower, driveTurn, intakePower, intakeTurn, rightPower, leftPower, max;
    private double motorMaxSpeed = 1800 ;
    private RelicFlipperState relicFlipperState = RelicFlipperState.Down;
    private RelicGrabberState relicGrabberState = RelicGrabberState.In;
    private long startTime;
    private boolean decelerating = false; //put in ROBOT

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

        drivePower = -gamepad1.left_stick_y;

        if (Math.abs(gamepad1.left_stick_y) > .15) {
            driveTurn = -(.65 * gamepad1.right_stick_x);
        } else {
            driveTurn = -(.7* gamepad1.right_stick_x);

        }



        intakePower = gamepad2.right_stick_y;
        intakeTurn = gamepad2.right_stick_x;


        //driving
        if (gamepad1.left_bumper) {//slow button
            rightBack.setPower((drivePower + 1.5*driveTurn) * .3);
            rightFront.setPower((drivePower + 1.5*driveTurn) * .3);
            leftBack.setPower((drivePower - 1.5*driveTurn) * .3);
            leftFront.setPower((drivePower - 1.5*driveTurn) * .3);
        } else if (drivePower <= .05 && drivePower >= -.05 && driveTurn <= .05 && driveTurn >= -.05) { //CASTED ALL DC MOTORS AD DCMOTOREX IN Robot CLASS
//            rightBack.setPower(Range.clip(rightBack.getPower() - .02, 0, motorMaxSpeed));
//            leftBack.setPower(Range.clip(leftBack.getPower() - .02, 0, motorMaxSpeed));
//            leftFront.setPower(Range.clip(leftFront.getPower() - .02, 0, motorMaxSpeed));
//            rightFront.setPower(Range.clip(rightFront.getPower() - .02, 0, motorMaxSpeed));
//            telemetry.addData("Decelerating",rightBack.getPower());
            rightBack.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            leftFront.setPower(0);
        } else if (gamepad1.right_bumper) {

        } else{
            rightPower = drivePower + driveTurn;
            leftPower = drivePower - driveTurn;
            max = Math.max(Math.abs(rightPower), Math.abs(leftPower));
            if (max > 1) {
                rightBack.setPower(rightPower/max);
                rightFront.setPower(rightPower/max);
                leftBack.setPower(leftPower/max);
                leftFront.setPower(leftPower/max);
            } else {
                rightBack.setPower(rightPower);
                rightFront.setPower(rightPower);
                leftBack.setPower(leftPower);
                leftFront.setPower(leftPower);
            }
//            rightBack.setPower(((0.91614053*drivePower - 0.04007548) + driveTurn));
//            rightFront.setPower(((0.91606515*drivePower - 0.03991444) + driveTurn));
//            leftBack.setPower(((0.91720407*drivePower - 0.03995359) - driveTurn));
//            leftFront.setPower(((0.91352592*drivePower - 0.03907521) - driveTurn));
        }



//-------------------------------------------------------------------
//THIS IS WHERE WE NEED THE DECELERATION THING PLZ
//-------------------------------------------------------------------


        //intake in
        if (gamepad2.a) {
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
        } else if (gamepad2.b) {//intake out
            intakeLeft.setPower(-.5);
            intakeRight.setPower(-.5);
        } else {
            intakeLeft.setPower(intakePower - intakeTurn);
            intakeRight.setPower(intakePower + intakeTurn);
        }


//        //lower slide
//        if (gamepad2.x) {
//            slideLeft.setPower(.65);
//            slideRight.setPower(.65);
//        } else if (gamepad2.y) { // raise slide
//            slideLeft.setPower(-.99);
//            slideRight.setPower(-.99);
//        } else {
//            slideLeft.setPower(0);
//            slideRight.setPower(0);
//        }

        //lower slide
        if (gamepad2.x) {
            slideState = 0;
        } else if (gamepad2.y) { // raise slide
            slideState = 1;
        }

        //AUTOMATIC SLIDE POSITIONS??
        switch (slideState) {
            case -1:
                slideLeft.setPower(0);
                slideRight.setPower(0);
                break;
            case 0:
                if (resetPosition) {
                    flipLeft.setPosition(flipLeftDown);
                    flipRight.setPosition(flipRightDown);
                    startTime = System.currentTimeMillis();
                    resetPosition = false;

                    slideLeft.setTargetPosition(0);
                    slideRight.setTargetPosition(0);

                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    slideLeft.setPower(.9);
                    slideRight.setPower(.9);
                }
                if ((slideRight.isBusy() || slideLeft.isBusy()) && (System.currentTimeMillis() - startTime < 1200)) {
                    break;
                } else {
                    slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resetPosition = true;
                    slideState = -1;
                }
                break;
            case 1:
                if (resetPosition) {
                    flipLeft.setPosition(flipLeftMiddle);
                    flipRight.setPosition(flipRightMiddle);
                    startTime = System.currentTimeMillis();
                    resetPosition = false;

                    slideLeft.setTargetPosition(-1200);
                    slideRight.setTargetPosition(-1200);

                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    slideLeft.setPower(.9);
                    slideRight.setPower(.9);
                }
                if ((slideRight.isBusy() || slideLeft.isBusy()) && (System.currentTimeMillis() - startTime < 1200)) {
                    break;
                } else {
                    resetPosition = true;
                    slideState = -1;
                }
                break;
        }

        //flip tray
        if (gamepad2.dpad_up) {
            flipLeft.setPosition(flipLeftUp);
            flipRight.setPosition(flipRightUp);
        } else if (gamepad2.dpad_down) {//lower tray
            flipLeft.setPosition(flipLeftDown);
            flipRight.setPosition(flipRightDown);
        } else if (gamepad2.dpad_left) {
            flipLeft.setPosition(flipLeftMiddle);
            flipRight.setPosition(flipRightMiddle);
        }

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

        //relic flipper
        if (gamepad2.left_stick_button) {
            relicFlipper.setPosition(relicFlipperUp);
            relicFlipperMidding = false;
        } else if (gamepad2.dpad_right) {
            relicFlipperMidding = true;
            startMidding = System.currentTimeMillis();
            relicFlipper.setPosition(relicFlipperMid);
        } else if (gamepad2.start) {
            relicFlipper.setPosition(relicFlipperDown);
            relicFlipperMidding = false;
        }

//        if (gamepad2.start) {
//            relicFlipper.setPosition(relicFlipperDown);
//        } else if (gamepad2.dpad_right) {
//            relicFlipper.setPosition(relicFlipperMid);
//        } else if (gamepad2.left_stick_button) {
//            relicFlipper.setPosition(relicFlipperUp);
//        }

        if (relicFlipperMidding) {
            relicFlipper.setPosition(Range.clip(relicFlipperUp - (System.currentTimeMillis() - startMidding)*0.0002725, relicFlipperMid, relicFlipperUp));
        }

        //relic grabber
        if (gamepad2.left_bumper) {
            relicGrabberOuting = true;
            startOuting = System.currentTimeMillis();
//            relicGrabberPosition = 0;
            relicGrabber.setPosition(relicGrabberOut);
        } else if (gamepad2.right_bumper) {
//            relicGrabberPosition = -1;
            relicGrabber.setPosition(relicGrabberIn);
            relicGrabberOuting = false;
        }

        if (relicGrabberOuting) {
            relicGrabber.setPosition(Range.clip(relicGrabberIn - (System.currentTimeMillis() - startOuting)*.0004, relicGrabberOut, relicGrabberIn));
        }
//        switch (relicGrabberPosition) {
//            case -1:
//                relicGrabber.setPosition(relicGrabberIn);
//                break;
//            case 0:
//                if(resetPosition){
//                    startTime=System.currentTimeMillis();
//                    resetPosition=false;
//                }
//                relicGrabber.setPosition(relicGrabberIn - .1);
//                if (System.currentTimeMillis() - startTime > 150) {
//                    resetPosition = true;
//                    relicGrabberPosition = 1;
//                }
//                break;
//            case 1:
//                if(resetPosition){
//                    startTime=System.currentTimeMillis();
//                    resetPosition=false;
//                }
//                relicGrabber.setPosition(relicGrabberIn - .2);
//                if (System.currentTimeMillis() - startTime > 150) {
//                    resetPosition = true;
//                    relicGrabberPosition = 2;
//                }
//                break;
//            case 2:
//                if(resetPosition){
//                    startTime=System.currentTimeMillis();
//                    resetPosition=false;
//                }
//                relicGrabber.setPosition(relicGrabberIn - .3);
//                if (System.currentTimeMillis() - startTime > 150) {
//                    resetPosition = true;
//                    relicGrabberPosition = 3;
//                }
//                break;
//            case 3:
//                if(resetPosition){
//                    startTime=System.currentTimeMillis();
//                    resetPosition=false;
//                }
//                relicGrabber.setPosition(relicGrabberIn - .4);
//                if (System.currentTimeMillis() - startTime > 150) {
//                    resetPosition = true;
//                    relicGrabberPosition = 4;
//                }
//                break;
//            case 4:
//                if(resetPosition){
//                    startTime=System.currentTimeMillis();
//                    resetPosition=false;
//                }
//                relicGrabber.setPosition(relicGrabberIn - .5);
//                if (System.currentTimeMillis() - startTime > 150) {
//                    resetPosition = true;
//                    relicGrabberPosition = 5;
//                }
//                break;
//            case 5:
//                relicGrabber.setPosition(relicGrabberOut);
//                break;
//        }

//        if (relicGrabberOuting) {
//            if (relicGrabberCurrent > relicGrabberOut) {
//                relicGrabberCurrent += .1;
//            } else {
//                relicGrabberOuting = false;
//            }
//        }
        telemetry.addData("hey", relicFlipperState.toString());
        telemetry.addData("hey", relicGrabberState.toString());

        telemetry.addData("currentPosition", relicGrabber.getPosition());

        //set relic servos
//        if (relicFlipperState == RelicFlipperState.Down) {
//            relicFlipper.setPosition(relicFlipperDown);
//        } if (relicFlipperState == RelicFlipperState.Middle) {
//            relicFlipper.setPosition(relicFlipperMid);
//        } if (relicFlipperState == RelicFlipperState.Up) {
//            relicFlipper.setPosition(relicFlipperUp);
//        }
//
//        if (relicGrabberState == RelicGrabberState.In) {
//            relicGrabber.setPosition(relicGrabberIn);
//        } if (relicGrabberState == RelicGrabberState.Out) {
//            relicGrabber.setPosition(relicGrabberOut);
//        }
        //
    }
}
