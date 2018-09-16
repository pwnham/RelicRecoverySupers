package org.firstinspires.ftc.teamcode.Supers.Auto;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Supers.Robot;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;

/**
 * Created by moham on 1/27/18.
 */
@Autonomous
public class RedSideCornerAuto extends Robot {

    private enum RobotState {//list states here
        DetectJewel, DeployArm, Done, LiftArm, DriveForward, KnockJewel, TurnBack, TurnToCrypto, DriveToCrypto, LiftFlipper, DriveAwayFromCrypto, NudgeGlyph, IntakeAgain, LowerFlipper, Park, TurnToKey, OuttakeGlyph, Wait
    }

    private RobotState robotState=RobotState.DetectJewel;//initialize start state here

    private long startTime;


    @Override
    public void init() {
        super.init();
        super.initAutonomous();
    }

    @Override
    public void loop() {
        switch(robotState) {
            case DetectJewel:
                if(resetPositionJewel){
                    startTime=System.currentTimeMillis();
                    resetPositionJewel=false;
                }
                if (System.currentTimeMillis() - startTime > 10000) {
                    columnKey = "CENTER";
                    jewelArm.setPosition(jewelArmUp);
                    robotState = RobotState.TurnToKey;
                    resetPosition = true;
                }
                if (!detected) {
                    if (vuforia.getFrameQueue().peek() != null) {
                        try {
                            Image img = getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565);
//                Mat cvImg = imageToMat(img);
                            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
                            bm.copyPixelsFromBuffer(img.getPixels());
                            Mat mat = new Mat();
                            Utils.bitmapToMat(bm, mat);
//                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2RGBA);

                            //for processing on vuMarks
                            for (VuforiaTrackable track : relicTrackables) {
                                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) track.getListener()).getRawPose();

                                if (pose != null) {
                                    Matrix34F rawPose = new Matrix34F();
                                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                                    rawPose.setData(poseData);

                                    Vec2F pointCenter = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(0, 0, 0));
                                    //left jewel
                                    Vec2F leftJewelTopLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(140, -90, 0));
                                    Vec2F leftJewelTopRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(160, -90, 0));
                                    Vec2F leftJewelBottomLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(140, -120, 0));
                                    Vec2F leftJewelBottomRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(160, -120, 0));
                                    //right jewel
//                        Vec2F pointLeftJewel = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(152, 0, 0));
//                        Vec2F pointLeftJewel = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(152, 0, 0));
//                        Vec2F pointLeftJewel = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(152, 0, 0));
//                        Vec2F pointRightJewel = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(0, 0, 0));

//                        Vec2F point2 = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(0, 0, 0));

                                    Imgproc.circle(mat, new Point(pointCenter.getData()[0], pointCenter.getData()[1]), 0, new Scalar(255, 0, 0), 5);
                                    Imgproc.circle(mat, new Point(pointCenter.getData()[0], pointCenter.getData()[1]), 50, new Scalar(255, 0, 0), 5);
                                    Imgproc.circle(mat, new Point(leftJewelTopLeft.getData()[0], leftJewelTopLeft.getData()[1]), 0, new Scalar(0, 255, 0), 5);
                                    Imgproc.circle(mat, new Point(leftJewelTopRight.getData()[0], leftJewelTopRight.getData()[1]), 0, new Scalar(0, 255, 0), 5);
                                    Imgproc.circle(mat, new Point(leftJewelBottomLeft.getData()[0], leftJewelBottomLeft.getData()[1]), 0, new Scalar(0, 255, 0), 5);
                                    Imgproc.circle(mat, new Point(leftJewelBottomRight.getData()[0], leftJewelBottomRight.getData()[1]), 0, new Scalar(0, 255, 0), 5);

                                    //crop image for left jewel
                                    Rect leftJewel = new Rect((int) leftJewelTopLeft.getData()[0], (int) leftJewelTopLeft.getData()[1], 50, 50);
                                    Mat croppedLeftJewel = new Mat(mat, leftJewel);
                                    Scalar averageLeft = Core.mean(croppedLeftJewel);
                                    if (averageLeft.val[0] > averageLeft.val[2]) {
                                        jewelColor = "RED";
                                        robotState = RobotState.DeployArm;
                                        resetPosition = true;
                                    } else if (averageLeft.val[2] > averageLeft.val[0]) {
                                        jewelColor = "BLUE";
                                        robotState = RobotState.DeployArm;
                                        resetPosition = true;
                                    }

                                }
                            }

                            try {
//                    Imgproc.cvtColor(seedsImage, tmp, Imgproc.COLOR_RGB2BGRA);
                                bm = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);
                                Utils.matToBitmap(mat, bm);
                                surfaceView.updateBitmap(bm);
                            } catch (CvException e) {
                                Log.d("Exception", e.getMessage());
                            }

                        } catch (Exception e) {
                            e.printStackTrace();
                        }

                    }

                    vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        columnKey = vuMark.toString();
                    }
                }
                break;
            case DeployArm:
                if(resetPosition){
                    startTime=System.currentTimeMillis();
                    resetPosition=false;
                }
                telemetry.addData("Left Jewel Color", jewelColor);
                jewelArm.setPosition(jewelArmDown);
                if (System.currentTimeMillis() - startTime > 1000) {
                    robotState = RobotState.KnockJewel;
                    resetPosition = true;
                }
                break;
            case KnockJewel:
                if(resetPosition){
                    startTime=System.currentTimeMillis();
                    resetPosition=false;
                }
                telemetry.addData("Left Jewel Color", jewelColor);
                if (jewelColor == "RED") {
                    jewelTurn.setPosition(jewelTurnBack);
                } else if (jewelColor == "BLUE") {
                    jewelTurn.setPosition(jewelTurnFront - .14);
                }
                if (System.currentTimeMillis() - startTime > 1000) {
                    robotState = RobotState.LiftArm;
                    resetPosition = true;
                }
                break;
//            case KnockJewel:
//                if(resetPosition){
//                    startTime=System.currentTimeMillis();
//                    resetPosition=false;
//
//                    if (jewelColor == "BLUE") {
//                        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                        leftBack.setTargetPosition(-250);
//                        leftFront.setTargetPosition(-250);
//                        rightBack.setTargetPosition(250);
//                        rightFront.setTargetPosition(250);
//
//                        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                        leftBack.setPower(.2);
//                        leftFront.setPower(.2);
//                        rightBack.setPower(.2);
//                        rightFront.setPower(.2);
//                    } else if (jewelColor == "RED") {
//                        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                        leftBack.setTargetPosition(250);
//                        leftFront.setTargetPosition(250);
//                        rightBack.setTargetPosition(-250);
//                        rightFront.setTargetPosition(-250);
//
//                        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                        leftBack.setPower(.2);
//                        leftFront.setPower(.2);
//                        rightBack.setPower(.2);
//                        rightFront.setPower(.2);
//                    }
//                }
//                if (leftBack.isBusy()) {
//                    break;
//                } else {
//                    robotState = RobotState.LiftArm;
//                    resetPosition = true;
//                }
//                break;
            case LiftArm:
                if(resetPosition){
                    startTime=System.currentTimeMillis();
                    resetPosition=false;
                }
                telemetry.addData("Left Jewel Color", jewelColor);
                jewelArm.setPosition(jewelArmUp);
                if (System.currentTimeMillis() - startTime > 1000) {
                    robotState = RobotState.TurnBack;
                    resetPosition = true;
                }
                break;
            case TurnBack:
                if(resetPosition){
                    startTime=System.currentTimeMillis();
                    resetPosition=false;
                }
                telemetry.addData("Left Jewel Color", jewelColor);
                jewelTurn.setPosition(jewelTurnCenter);
                if (System.currentTimeMillis() - startTime > 1000) {
                    robotState = RobotState.TurnToKey;
                    resetPosition = true;
                }
                break;
//            case TurnBack:
//                if(resetPosition){
//                    startTime=System.currentTimeMillis();
//                    resetPosition=false;
//
//                    if (jewelColor == "BLUE") {
//                        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                        leftBack.setTargetPosition(250);
//                        leftFront.setTargetPosition(250);
//                        rightBack.setTargetPosition(-250);
//                        rightFront.setTargetPosition(-250);
//
//                        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                        leftBack.setPower(.2);
//                        leftFront.setPower(.2);
//                        rightBack.setPower(.2);
//                        rightFront.setPower(.2);
//                    } else if (jewelColor == "RED") {
//                        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                        leftBack.setTargetPosition(-250);
//                        leftFront.setTargetPosition(-250);
//                        rightBack.setTargetPosition(250);
//                        rightFront.setTargetPosition(250);
//
//                        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                        leftBack.setPower(.2);
//                        leftFront.setPower(.2);
//                        rightBack.setPower(.2);
//                        rightFront.setPower(.2);
//                    }
//                }
//                if (leftBack.isBusy()) {
//                    break;
//                } else {
//                    robotState = RobotState.DriveForward;
//                    resetPosition = true;
//                }
//                break;
            case TurnToKey:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;
                    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    if (columnKey == "CENTER") {
                        leftBack.setTargetPosition(-110);
                        leftFront.setTargetPosition(-110);
                        rightBack.setTargetPosition(110);
                        rightFront.setTargetPosition(110);
                    } else if (columnKey == "LEFT") {
                        leftBack.setTargetPosition(-150);
                        leftFront.setTargetPosition(-150);
                        rightBack.setTargetPosition(150);
                        rightFront.setTargetPosition(150);
                    } else if (columnKey =="RIGHT") {
                        leftBack.setTargetPosition(-55);
                        leftFront.setTargetPosition(-55);
                        rightBack.setTargetPosition(55);
                        rightFront.setTargetPosition(55);
                    } else {
                        leftBack.setTargetPosition(-110);
                        leftFront.setTargetPosition(-110);
                        rightBack.setTargetPosition(110);
                        rightFront.setTargetPosition(110);
                    }

                    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.2);
                    leftFront.setPower(.2);
                    rightBack.setPower(.2);
                    rightFront.setPower(.2);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 5000)) {
                    break;
                } else {
                    robotState = RobotState.DriveForward;
                    resetPosition = true;
                }
                break;
            case DriveForward:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;
                    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                    if (columnKey == "CENTER") {
                        leftBack.setTargetPosition(1370);
                        leftFront.setTargetPosition(1370);
                        rightBack.setTargetPosition(1370);
                        rightFront.setTargetPosition(1370);
                    } else if (columnKey == "LEFT") {
                        leftBack.setTargetPosition(1435);
                        leftFront.setTargetPosition(1435);
                        rightBack.setTargetPosition(1435);
                        rightFront.setTargetPosition(1435);
                    } else if (columnKey =="RIGHT") {
                        leftBack.setTargetPosition(1340);
                        leftFront.setTargetPosition(1340);
                        rightBack.setTargetPosition(1340);
                        rightFront.setTargetPosition(1340);
                    }


                    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.2);
                    leftFront.setPower(.2);
                    rightBack.setPower(.2);
                    rightFront.setPower(.2);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 5000)) {
                    break;
                } else {
                    robotState = RobotState.OuttakeGlyph;
                    resetPosition = true;
                }
                break;
            case OuttakeGlyph:
                if(resetPosition){
                    startTime=System.currentTimeMillis();
                    resetPosition=false;
                }
                intakeLeft.setPower(-1);
                intakeRight.setPower(-1);
                if (System.currentTimeMillis() - startTime > 500) {
                    robotState = RobotState.DriveToCrypto;
                    resetPosition = true;
                }
                break;
            case DriveToCrypto:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;
                    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftBack.setTargetPosition(-400);
                    leftFront.setTargetPosition(-400);
                    rightBack.setTargetPosition(-400);
                    rightFront.setTargetPosition(-400);

                    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.2);
                    leftFront.setPower(.2);
                    rightBack.setPower(.2);
                    rightFront.setPower(.2);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 5000)) {
                    break;
                } else {
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    robotState = RobotState.DriveAwayFromCrypto;
                    resetPosition = true;
                }
                break;
            case LiftFlipper:
                if(resetPosition){
                    startTime=System.currentTimeMillis();
                    resetPosition=false;
                }
                flipLeft.setPosition(flipLeftUp);
                flipRight.setPosition(flipRightUp);
                if (System.currentTimeMillis() - startTime > 1000) {
                    robotState = RobotState.DriveAwayFromCrypto;
                    resetPosition = true;
                }
                break;
            case DriveAwayFromCrypto:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;
                    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftBack.setTargetPosition(400);
                    leftFront.setTargetPosition(400);
                    rightBack.setTargetPosition(400);
                    rightFront.setTargetPosition(400);

                    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.2);
                    leftFront.setPower(.2);
                    rightBack.setPower(.2);
                    rightFront.setPower(.2);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 5000)) {
                    break;
                } else {
                    robotState = RobotState.NudgeGlyph;
                    resetPosition = true;
                }
                break;
            case LowerFlipper:
                if(resetPosition){
                    startTime=System.currentTimeMillis();
                    resetPosition=false;
                }
                flipLeft.setPosition(flipLeftDown);
                flipRight.setPosition(flipRightDown);
                if (System.currentTimeMillis() - startTime > 1000) {
                    robotState = RobotState.NudgeGlyph;
                    resetPosition = true;
                }
                break;
            case NudgeGlyph:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;
                    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftBack.setTargetPosition(-380);
                    leftFront.setTargetPosition(-380);
                    rightBack.setTargetPosition(-380);
                    rightFront.setTargetPosition(-380);

                    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.2);
                    leftFront.setPower(.2);
                    rightBack.setPower(.2);
                    rightFront.setPower(.2);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 5000)) {
                    break;
                } else {
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    robotState = RobotState.Done;
                    resetPosition = true;
                }
                break;
            case Park:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;
                    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftBack.setTargetPosition(350);
                    leftFront.setTargetPosition(350);
                    rightBack.setTargetPosition(350);
                    rightFront.setTargetPosition(350);

                    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.2);
                    leftFront.setPower(.2);
                    rightBack.setPower(.2);
                    rightFront.setPower(.2);
                }
                if (leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) {
                    break;
                } else {
                    robotState = RobotState.Done;
                    resetPosition = true;
                }
                break;
            case IntakeAgain:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;
                    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftBack.setTargetPosition(2000);
                    leftFront.setTargetPosition(2000);
                    rightBack.setTargetPosition(2000);
                    rightFront.setTargetPosition(2000);

                    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.5);
                    leftFront.setPower(.5);
                    rightBack.setPower(.5);
                    rightFront.setPower(.5);
                }
                if (leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) {
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                    break;
                } else {
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    robotState = RobotState.Done;
                    resetPosition = true;
                }
                break;
            case Done:
                rightBack.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                leftFront.setPower(0);
                telemetry.addData("DONE", "DONE");
                break;

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
