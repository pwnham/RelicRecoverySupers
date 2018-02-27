package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.os.Handler;
import android.os.Looper;
import android.support.annotation.Nullable;
import android.util.Log;
import android.view.View;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;

/**
 * Created by moham on 1/10/18.
 */
@Autonomous
public class JewelKnockBlueAuto extends OpMode{


    private enum RobotState {//list states here
        DetectJewel, DeployArm, Done, KnockJewel, ParkFromFar, LiftArm, Wait, ParkFromNear
    }

    private RobotState robotState=RobotState.DetectJewel;//initialize start state here

    VuforiaLocalizer vuforia;
    int cameraMonitorViewId;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    VuforiaTrackableDefaultListener listener;
    RelicRecoveryVuMark vuMark;
    Image rgb;
    Context context;
    public static int WIDTH=1280, HEIGHT=720;
    MySurfaceView surfaceView;

    Handler handler;

    DcMotor leftBack, leftFront, intakeLeft, intakeRight, rightFront, rightBack;
    Servo clampLeft, clampRight, mandibleLeft, mandibleRight, jewelArm;
    CRServo conveyorLeft, conveyorRight, lift1, lift2;
    public double grabRightOut = .7;
    public double grabRightIn = .9;
    public double grabLeftOut = .85;
    public double grabLeftIn = .65;
    public double mandibleRightOut = .4;
    public double mandibleRightIn = .8;
    public double mandibleLeftOut = .65;
    public double mandibleLeftIn = .4;
    public double jewelArmUp = 1;
    public double jewelArmDown = .325;
    public double conveyorLeftStart;
    public double conveyorRightStart;
    public double driveMultiplier = 1;
    private boolean detected = false;
    private String jewelColor;
    public boolean resetPosition = true;
    private long startTime;

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(context) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i("OpenCV", "OpenCV loaded successfully");
                    // mat = new Mat();
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    @Override
    public void init() {
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");

        conveyorRight = hardwareMap.crservo.get("conveyorRight");
        conveyorLeft = hardwareMap.crservo.get("conveyorLeft");
        lift1 = hardwareMap.crservo.get("liftTop");
        lift2 = hardwareMap.crservo.get("liftBottom");
        clampLeft = hardwareMap.servo.get("clampLeft");
        clampRight = hardwareMap.servo.get("clampRight");
        mandibleLeft = hardwareMap.servo.get("mandibleLeft");
        mandibleRight = hardwareMap.servo.get("mandibleRight");
        jewelArm = hardwareMap.servo.get("jewelArm");

//        conveyorLeftStart = conveyorLeft.getPosition();
//        conveyorRightStart = conveyorRight.getPosition();

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);


        rightBack.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        leftFront.setPower(0);
        intakeLeft.setPower(0);
        intakeRight.setPower(0);

        clampLeft.setPosition(grabLeftOut);
        clampRight.setPosition(grabRightOut);
        jewelArm.setPosition(jewelArmUp);

        handler= new Handler(Looper.getMainLooper());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AY1sGR7/////AAAAGUIEPxq1Q0OHlg+rWY6E5Kh4qSsw5c/A1GkGZpCg2zSlxfB97RdicKHfBIuhCnfS1Lx+HQL68oWdyLX5SGlgYVH1BW8TiWpRnNFQ4o0f1qwG5P67klb5EQilENwYt7EIBFTSjFOAWxX9vWyjhL+rW6u9k9dGnCut3ukqr+JML+UjSEXIAhXWEAVu86UH/Y70mx8cobzt3UqSv4PpNStZHPB6OakhBgIiCe2Leh04/kWJ+uAsIyO6bukEOilVcSC0A7LsbvNwOzbqPI1poFBqVXeEP5xqenaGIfgLAjj6g2b4sHXosmphZyAiL8v8XXrq/Uz3KTD1B8Lj9aLcs7c0mZ02yZnqCfOpIhYoUmKRVBgB";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();
        listener = (VuforiaTrackableDefaultListener) relicTrackables.get(0).getListener();

        context = hardwareMap.appContext;
        final Activity activity = (Activity) hardwareMap.appContext;

        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                LinearLayout l = (LinearLayout) activity.findViewById(R.id.cameraMonitorViewId);
                surfaceView = new MySurfaceView(context, WIDTH, HEIGHT);
                l.addView(surfaceView);
            }
        });

        if (!OpenCVLoader.initDebug()) {
            Log.d("OpenCV", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, context, mLoaderCallback);
        } else {
            Log.d("OpenCV", "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    @Override
    public void loop() {
        switch(robotState) {
            case DetectJewel:
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
                                    Vec2F leftJewelTopLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(180, -90, 0));
                                    Vec2F leftJewelTopRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(240, -90, 0));
                                    Vec2F leftJewelBottomLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(180, -150, 0));
                                    Vec2F leftJewelBottomRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(240, -150, 0));
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
                                    } else if (averageLeft.val[2] > averageLeft.val[0]) {
                                        jewelColor = "BLUE";
                                        robotState = RobotState.DeployArm;
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
                if (System.currentTimeMillis() - startTime < 250) {
                    if (jewelColor == "RED") {
                        rightBack.setPower(.5);
                        rightFront.setPower(-.5);
                        leftBack.setPower(-.5);
                        leftFront.setPower(-.5);
                    } else if (jewelColor == "BLUE") {
                        rightBack.setPower(-.5);
                        rightFront.setPower(.5);
                        leftBack.setPower(.5);
                        leftFront.setPower(.5);
                    }
                } else {
                    robotState = RobotState.Wait;
                    resetPosition = true;
                }
                break;
            case Wait:
                if(resetPosition){
                    startTime=System.currentTimeMillis();
                    resetPosition=false;
                }
                if (System.currentTimeMillis() - startTime > 500) {
                    robotState = RobotState.LiftArm;
                    resetPosition = true;
                }
                break;
            case LiftArm:
                if(resetPosition){
                    startTime=System.currentTimeMillis();
                    resetPosition=false;
                }
                telemetry.addData("Left Jewel Color", jewelColor);
                jewelArm.setPosition(jewelArmUp);
                if (System.currentTimeMillis() - startTime > 200) {
                    if (jewelColor == "RED") {
                        robotState = RobotState.Done;
                    } else if (jewelColor == "BLUE") {
                        robotState = RobotState.Done;
                    }
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
    }

    @Override
    public void stop() {
        handler.post(new Runnable() {
            public void run() {
                closeView(); //or whatever method you want to call thats currently not working
            }
        });
    }

    private void closeView() {
        surfaceView.setVisibility(View.GONE);
    }
    @Nullable
    public static Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {

        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }//if
        }//for

        return null;
    }

    public static Mat imageToMat(Image img) {
        Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(img.getPixels());
        Mat tmp = new Mat(img.getWidth(), img.getHeight(), CvType.CV_8UC4);
        Utils.bitmapToMat(bm, tmp);
        return tmp;
    }
}
