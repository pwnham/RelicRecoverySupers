package org.firstinspires.ftc.teamcode.Supers.Auto;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
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

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.ScrimRobot.driveState.TurnLeft;

/**
 * Created by moham on 1/27/18.
 */
@Autonomous
public class BlueSideCornerMultiAuto extends Robot {

    private enum RobotState {//list states here
        DetectJewel, DeployArm, Done, LiftArm, DriveForward, KnockJewel, TurnBack, TurnToCrypto, DriveToCrypto, LiftFlipper, DriveAwayFromCrypto, NudgeGlyph, IntakeAgain, LowerFlipper, Park, DriveBackFromIntakeAgain, LiftFlipperAgain, TurnAfterIntake, FinalPark, BackOutABit, FinalPush, FinalTurn, TurnLeft, TurnToKey, TurnBeforeIntake, BackOutABitBeforeIntake, Wait
    }

    private RobotState robotState=RobotState.DetectJewel;//initialize start state here

    private long startTime, startTimeIntake;
    private int intakeCount = 0;

    private ArrayList<Long> timeStamp = new ArrayList<Long>();
    private ArrayList<Double> intakeLeftPos = new ArrayList<Double>();
    private ArrayList<Double> intakeRightPos = new ArrayList<Double>();

    private boolean gyroInitialized = false;

    private double intakeLeftSum = 0, intakeRightSum = 0;
    @Override
    public void init() {
        super.init();
        super.initAutonomous();
    }

    @Override
    public void loop() {
        if (!gyroInitialized) {
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            gyroInitialized = true;
        }
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).toAngleUnit(AngleUnit.DEGREES);

        double[] intakeVelocities = getVelocity(intakeLeft, intakeRight);
        telemetry.addData("Intake Left Velocity", intakeVelocities[0]);
        telemetry.addData("Intake Right Velocity", intakeVelocities[1]);
        if (Math.abs(intakeVelocities[0]) > .025) {
            intakeLeftPos.add(intakeVelocities[0]);
            intakeRightPos.add(intakeVelocities[1]);
        }
        switch(robotState) {
            case DetectJewel:
                if(resetPositionJewel){
                    startTime=System.currentTimeMillis();
                    resetPositionJewel=false;
                }
                if (System.currentTimeMillis() - startTime > 10000) {
                    columnKey = "CENTER";
                    jewelArm.setPosition(jewelArmUp);
                    robotState = RobotState.DriveForward;
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
                                        telemetry.addData("ball", jewelColor);
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
                if (System.currentTimeMillis() - startTime > 700) {
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
                if (jewelColor == "BLUE") {
                    jewelTurn.setPosition(jewelTurnBack  + .35);
                } else if (jewelColor == "RED") {
                    jewelTurn.setPosition(jewelTurnFront);
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
                        leftBack.setTargetPosition(117);
                        leftFront.setTargetPosition(117);
                        rightBack.setTargetPosition(-117);
                        rightFront.setTargetPosition(-117);
                    } else if (columnKey == "LEFT") {
                        leftBack.setTargetPosition(55);
                        leftFront.setTargetPosition(55);
                        rightBack.setTargetPosition(-55);
                        rightFront.setTargetPosition(-55);
                    } else if (columnKey =="RIGHT") {
                        leftBack.setTargetPosition(170);
                        leftFront.setTargetPosition(170);
                        rightBack.setTargetPosition(-170);
                        rightFront.setTargetPosition(-170);
                    } else {
                        leftBack.setTargetPosition(250);
                        leftFront.setTargetPosition(250);
                        rightBack.setTargetPosition(-250);
                        rightFront.setTargetPosition(-250);
                    }

                    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.6);
                    leftFront.setPower(.6);
                    rightBack.setPower(.6);
                    rightFront.setPower(.6);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 2000)) {
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
                        leftBack.setTargetPosition(-1410);
                        leftFront.setTargetPosition(-1410);
                        rightBack.setTargetPosition(-1410);
                        rightFront.setTargetPosition(-1410);
                    } else if (columnKey == "LEFT") {
                        leftBack.setTargetPosition(-1385);
                        leftFront.setTargetPosition(-1385);
                        rightBack.setTargetPosition(-1385);
                        rightFront.setTargetPosition(-1385);
                    } else if (columnKey =="RIGHT") {
                        leftBack.setTargetPosition(-1490);
                        leftFront.setTargetPosition(-1490);
                        rightBack.setTargetPosition(-1490);
                        rightFront.setTargetPosition(-1490);
                    }

                    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.6);
                    leftFront.setPower(.6);
                    rightBack.setPower(.6);
                    rightFront.setPower(.6);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 2000)) {
                    break;
                } else {
                    robotState = RobotState.LiftFlipper;
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

                    leftBack.setPower(.6);
                    leftFront.setPower(.6);
                    rightBack.setPower(.6);
                    rightFront.setPower(.6);
                }
                if (leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) {
                    break;
                } else {
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

                    leftBack.setTargetPosition(200);
                    leftFront.setTargetPosition(200);
                    rightBack.setTargetPosition(200);
                    rightFront.setTargetPosition(200);

                    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.6);
                    leftFront.setPower(.6);
                    rightBack.setPower(.6);
                    rightFront.setPower(.6);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 1000)) {
                    break;
                } else {
                    robotState = RobotState.LowerFlipper;
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

                    leftBack.setTargetPosition(-200);
                    leftFront.setTargetPosition(-200);
                    rightBack.setTargetPosition(-200);
                    rightFront.setTargetPosition(-200);

                    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.6);
                    leftFront.setPower(.6);
                    rightBack.setPower(.6);
                    rightFront.setPower(.6);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 1000)) {
                    intakeLeft.setPower(-1);
                    intakeRight.setPower(-1);
                    break;
                } else {
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    robotState = RobotState.TurnLeft;
                    resetPosition = true;
                }
                break;
            case TurnLeft:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;
                    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftBack.setTargetPosition(-190);
                    leftFront.setTargetPosition(-190);
                    rightBack.setTargetPosition(190);
                    rightFront.setTargetPosition(190);

                    leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.6);
                    leftFront.setPower(.6);
                    rightBack.setPower(.6);
                    rightFront.setPower(.6);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 1000)) {
                    break;
                } else {
                    robotState = RobotState.BackOutABitBeforeIntake;
                    resetPosition = true;
                }
                break;
            case BackOutABitBeforeIntake:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;

                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftBack.setTargetPosition(200);
                    leftFront.setTargetPosition(200);
                    rightBack.setTargetPosition(200);
                    rightFront.setTargetPosition(200);

                    setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.6);
                    leftFront.setPower(.6);
                    rightBack.setPower(.6);
                    rightFront.setPower(.6);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 500)) {
                    break;
                } else {
                    robotState = RobotState.TurnBeforeIntake;
                    resetPosition = true;
                }
                break;
            case TurnBeforeIntake:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;

                    imu.initialize(imuparameters);

                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);

                    setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    try {
                        wait(500);
                    } catch (Exception e) {
                    }
                }
                if (columnKey == "CENTER") {
                    double error = angles.firstAngle - (22);
                    double power = .015 * error;
                    if (Math.abs(error) > 1) {
                        leftBack.setPower(power);
                        leftFront.setPower(power);
                        rightBack.setPower(-power);
                        rightFront.setPower(-power);
                    } else {
                        leftBack.setPower(0);
                        leftFront.setPower(0);
                        rightBack.setPower(0);
                        rightFront.setPower(0);
                        robotState = RobotState.IntakeAgain;
                        resetPosition = true;
                    }
                } else if (columnKey == "LEFT") {
                    double error = angles.firstAngle - (-25);
                    double power = .02 * error;
                    if (Math.abs(error) > 1) {
                        leftBack.setPower(power);
                        leftFront.setPower(power);
                        rightBack.setPower(-power);
                        rightFront.setPower(-power);
                    } else {
                        leftBack.setPower(0);
                        leftFront.setPower(0);
                        rightBack.setPower(0);
                        rightFront.setPower(0);
                        robotState = RobotState.IntakeAgain;
                        resetPosition = true;
                    }
                } else if (columnKey == "RIGHT") {
                    double error = angles.firstAngle - (20);
                    double power = .02 * error;
                    if (Math.abs(error) > 1) {
                        leftBack.setPower(power);
                        leftFront.setPower(power);
                        rightBack.setPower(-power);
                        rightFront.setPower(-power);
                    } else {
                        leftBack.setPower(0);
                        leftFront.setPower(0);
                        rightBack.setPower(0);
                        rightFront.setPower(0);
                        robotState = RobotState.IntakeAgain;
                        resetPosition = true;
                    }
                }
                break;
            case IntakeAgain:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;

                    imu.initialize(imuparameters);

                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftBack.setTargetPosition(1850);
                    leftFront.setTargetPosition(1850);
                    rightBack.setTargetPosition(1850);
                    rightFront.setTargetPosition(1850);

                    setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.5);
                    leftFront.setPower(.5);
                    rightBack.setPower(.5);
                    rightFront.setPower(.5);
                    try {
                        wait(500);
                    } catch (Exception e) {
                    }
                }
//                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 7000)) {
                if (System.currentTimeMillis() - startTime < 5500) {
                    switch (intakeState) {
                        case 0:
                            if (resetIntake) {
                                startTimeIntake = System.currentTimeMillis();
                                resetIntake = false;
                            }
                            if (System.currentTimeMillis() - startTimeIntake > 800) {
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
                } else {
                    robotState = RobotState.BackOutABit;
                    resetPosition = true;
                }
                break;
            case BackOutABit:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;

                    leftBack.setTargetPosition(1400);
                    leftFront.setTargetPosition(1400);
                    rightBack.setTargetPosition(1400);
                    rightFront.setTargetPosition(1400);

                    setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.6);
                    leftFront.setPower(.6);
                    rightBack.setPower(.6);
                    rightFront.setPower(.6);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 2000)) {
                    break;
                } else {
                    robotState = RobotState.TurnAfterIntake;
                    resetPosition = true;
                }
                break;
            case TurnAfterIntake:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;

                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);

                    setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (columnKey == "CENTER") {
//                    double error = angles.firstAngle - (14);
//                    double power = .02 * error;
//                    if (Math.abs(error) > 1) {
//                        leftBack.setPower(power);
//                        leftFront.setPower(power);
//                        rightBack.setPower(-power);
//                        rightFront.setPower(-power);
//                    } else {
//                        leftBack.setPower(0);
//                        leftFront.setPower(0);
//                        rightBack.setPower(0);
//                        rightFront.setPower(0);
//                        robotState = RobotState.DriveBackFromIntakeAgain;
//                        resetPosition = true;
//                    }
                    robotState = RobotState.DriveBackFromIntakeAgain;
                    resetPosition = true;
                } else if (columnKey == "LEFT") {
                    double error = angles.firstAngle - (-20);
                    double power = .02 * error;
                    if (Math.abs(error) > 1) {
                        leftBack.setPower(power);
                        leftFront.setPower(power);
                        rightBack.setPower(-power);
                        rightFront.setPower(-power);
                    } else {
                        leftBack.setPower(0);
                        leftFront.setPower(0);
                        rightBack.setPower(0);
                        rightFront.setPower(0);
                        robotState = RobotState.DriveBackFromIntakeAgain;
                        resetPosition = true;
                    }
                } else if (columnKey == "RIGHT") {
                    double error = angles.firstAngle - (2);
                    double power = .02 * error;
                    if (Math.abs(error) > 1) {
                        leftBack.setPower(power);
                        leftFront.setPower(power);
                        rightBack.setPower(-power);
                        rightFront.setPower(-power);
                    } else {
                        leftBack.setPower(0);
                        leftFront.setPower(0);
                        rightBack.setPower(0);
                        rightFront.setPower(0);
                        robotState = RobotState.DriveBackFromIntakeAgain;
                        resetPosition = true;
                    }
                }
                break;
            case DriveBackFromIntakeAgain:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    flipLeft.setPosition(flipLeftMiddle);
                    flipRight.setPosition(flipRightMiddle);
                    resetPosition = false;

                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//                    slideLeft.setTargetPosition(-600);
//                    slideRight.setTargetPosition(-600);

                    if (columnKey == "CENTER") {
                        leftBack.setTargetPosition(-1300);
                        leftFront.setTargetPosition(-1300);
                        rightBack.setTargetPosition(-1300);
                        rightFront.setTargetPosition(-1300);
                    } else if (columnKey == "LEFT") {
                        leftBack.setTargetPosition(-1200);
                        leftFront.setTargetPosition(-1200);
                        rightBack.setTargetPosition(-1200);
                        rightFront.setTargetPosition(-1200);
                    } else if (columnKey =="RIGHT") {
                        leftBack.setTargetPosition(-1300);
                        leftFront.setTargetPosition(-1300);
                        rightBack.setTargetPosition(-1300);
                        rightFront.setTargetPosition(-1300);
                    }

                    setMode(DcMotor.RunMode.RUN_TO_POSITION);

//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.6);
                    leftFront.setPower(.6);
                    rightBack.setPower(.6);
                    rightFront.setPower(.6);

                }
                if (System.currentTimeMillis() - startTime < 250) {
                    intakeLeft.setPower(-1);
                    intakeRight.setPower(-1);
                } else if (System.currentTimeMillis() - startTime > 250 && System.currentTimeMillis() - startTime < 1000) {
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                } else {
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 2500)) {
                    break;
                } else {
//                    intakeLeft.setPower(0);
//                    intakeRight.setPower(0);
                    robotState = RobotState.LiftFlipperAgain;
                    resetPosition = true;
                }
                break;
            case LiftFlipperAgain:
                if(resetPosition){
                    startTime=System.currentTimeMillis();
                    resetPosition=false;
                }
                flipLeft.setPosition(flipLeftUp);
                flipRight.setPosition(flipRightUp);
                if (System.currentTimeMillis() - startTime > 750) {
                    robotState = RobotState.FinalTurn;
                    resetPosition = true;
                }
                break;
            case FinalTurn:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;

                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    if (columnKey == "CENTER") {
                        leftBack.setTargetPosition(-180);
                        leftFront.setTargetPosition(-180);
                        rightBack.setTargetPosition(180);
                        rightFront.setTargetPosition(180);
                    } else if (columnKey == "LEFT") {
                        leftBack.setTargetPosition(-175);
                        leftFront.setTargetPosition(-175);
                        rightBack.setTargetPosition(175);
                        rightFront.setTargetPosition(175);
                    } else if (columnKey =="RIGHT") {
                        leftBack.setTargetPosition(150);
                        leftFront.setTargetPosition(150);
                        rightBack.setTargetPosition(-150);
                        rightFront.setTargetPosition(-150);
                    }

                    setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.6);
                    leftFront.setPower(.6);
                    rightBack.setPower(.6);
                    rightFront.setPower(.6);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 1500)) {
                    break;
                } else {
                    robotState = RobotState.FinalPush;
                    resetPosition = true;
                }
                break;
            case FinalPush:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;

                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftBack.setTargetPosition(-200);
                    leftFront.setTargetPosition(-200);
                    rightBack.setTargetPosition(-200);
                    rightFront.setTargetPosition(-200);

                    setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.6);
                    leftFront.setPower(.6);
                    rightBack.setPower(.6);
                    rightFront.setPower(.6);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 1500)) {
                    break;
                } else {
                    robotState = RobotState.FinalPark;
                    resetPosition = true;
                }
                break;
            case FinalPark:
                if (resetPosition) {
                    startTime = System.currentTimeMillis();
                    resetPosition = false;

                    setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftBack.setTargetPosition(300);
                    leftFront.setTargetPosition(300);
                    rightBack.setTargetPosition(300);
                    rightFront.setTargetPosition(300);

                    setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBack.setPower(.6);
                    leftFront.setPower(.6);
                    rightBack.setPower(.6);
                    rightFront.setPower(.6);
                }
                if ((leftBack.isBusy() || leftFront.isBusy() || rightBack.isBusy() || rightFront.isBusy()) && (System.currentTimeMillis() - startTime < 1500)) {
                    break;
                } else {
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

    public boolean intakeOutIn() {
        telemetry.addData("intakeCount", intakeCount++);
        switch (intakeState) {
            case 0:
                if (resetIntake) {
                    startTimeIntake = System.currentTimeMillis();
                    resetIntake = false;
                }
                if (System.currentTimeMillis() - startTimeIntake > 750) {
                    intakeState = 1;
                    resetIntake = true;
                } else {
                    intakeLeft.setPower(1);
                    intakeRight.setPower(1);
                }
                break;
            case 1:
                if (resetIntake) {
                    startTimeIntake = System.currentTimeMillis();
                    resetIntake = false;
                }
                if (System.currentTimeMillis() - startTimeIntake > 250) {
                    intakeState = 1;
                    resetIntake = true;
                } else {
                    intakeLeft.setPower(-1);
                    intakeRight.setPower(-1);
                }
                break;

        }
        telemetry.addData("intakeLeft", intakeLeft.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("intakeRight", intakeRight.getVelocity(AngleUnit.DEGREES));

        return true;
    }

    @Override
    public void stop() {

        Log.d("timeStamp", timeStamp.toString());

        Log.d("intakeLeftPos", intakeLeftPos.toString());
        Log.d("intakeRightPos", intakeRightPos.toString());

    }



}

