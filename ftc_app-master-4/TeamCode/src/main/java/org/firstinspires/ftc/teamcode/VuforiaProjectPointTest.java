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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;

/**
 * Created by moham on 11/13/17.
 */
@TeleOp
public class VuforiaProjectPointTest extends OpMode{
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
                        Vec2F leftJewelBottomLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(140, -150, 0));
                        Vec2F leftJewelBottomRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(160, -150, 0));
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
                            telemetry.addData("Left Jewel Color", "RED");

                        } else if (averageLeft.val[2] > averageLeft.val[0]) {
                            telemetry.addData("Left Jewel Color", "BLUE");
                        }

                    }
                }

                try {
//                    Imgproc.cvtColor(seedsImage, tmp, Imgproc.COLOR_RGB2BGRA);
                    bm = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);
                    Utils.matToBitmap(mat, bm);
                    surfaceView.updateBitmap(bm);
                }
                catch (CvException e){Log.d("Exception",e.getMessage());}

            } catch (Exception e) {
                e.printStackTrace();
            }

        }

        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
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