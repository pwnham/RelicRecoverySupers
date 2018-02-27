package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.support.annotation.Nullable;
import android.util.Log;
import android.view.SurfaceView;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
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
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;

/**
 * Created by moham on 9/8/17.
 */

@TeleOp
public class TestAuto extends OpMode {
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
    //Mat mat;

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
        //cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AY1sGR7/////AAAAGUIEPxq1Q0OHlg+rWY6E5Kh4qSsw5c/A1GkGZpCg2zSlxfB97RdicKHfBIuhCnfS1Lx+HQL68oWdyLX5SGlgYVH1BW8TiWpRnNFQ4o0f1qwG5P67klb5EQilENwYt7EIBFTSjFOAWxX9vWyjhL+rW6u9k9dGnCut3ukqr+JML+UjSEXIAhXWEAVu86UH/Y70mx8cobzt3UqSv4PpNStZHPB6OakhBgIiCe2Leh04/kWJ+uAsIyO6bukEOilVcSC0A7LsbvNwOzbqPI1poFBqVXeEP5xqenaGIfgLAjj6g2b4sHXosmphZyAiL8v8XXrq/Uz3KTD1B8Lj9aLcs7c0mZ02yZnqCfOpIhYoUmKRVBgB";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
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

//        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(context) {
//            @Override
//            public void onManagerConnected(int status) {
//                switch (status) {
//                    case LoaderCallbackInterface.SUCCESS:
//                    {
//                        Log.i("OpenCV", "OpenCV loaded successfully");
//                    } break;
//                    default:
//                    {
//                        super.onManagerConnected(status);
//                    } break;
//                }
//            }
//        };

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


                //for processing on vuMarks
//                for (VuforiaTrackable track : relicTrackables) {
//                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) track.getListener()).getRawPose();
//
//                    if (pose != null) {
//                        Matrix34F rawPose = new Matrix34F();
//                        float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
//                        rawPose.setData(poseData);
//
//                        Vec2F point = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(0, 0, 0));
//
//                        Imgproc.circle(mat, new Point(point.getData()[0], point.getData()[1]), 50, new Scalar(0, 255, 0), 5);
//                    }
//                }

                Mat gray = new Mat();
                Mat circles = new Mat();
                Imgproc.cvtColor(mat, gray, Imgproc.COLOR_RGBA2GRAY);
                Imgproc.blur(gray, gray, new Size(9, 9));
                Imgproc.morphologyEx(gray, gray, Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(50, 50)));

                Imgproc.HoughCircles(gray,circles,Imgproc.CV_HOUGH_GRADIENT, 2//resolution modifier
                        ,gray.rows()/4//minimum distance
                        ,200//param1, canny algorithm top param unused????????
                        ,40//param2, circle accumulation param
                        ,20//min radius
                        ,0//max radius
                );
                if (circles.cols() > 0) {
                    for (int i = 0;i<circles.cols();i++) {
                        double vCircle[] = circles.get(0, i);

                        if (vCircle == null) {
                            break;
                        }

                        Point pt = new Point(Math.round(vCircle[0]), Math.round(vCircle[1]));
                        int radius = (int)Math.round(vCircle[2]);

                       // Imgproc.circle(mat, pt, radius, new Scalar(0, 255, 0), 5);
                        Imgproc.circle(gray, pt, radius, new Scalar(0, 255, 0), 5);

                    }
                }

                try {
//                    Imgproc.cvtColor(seedsImage, tmp, Imgproc.COLOR_RGB2BGRA);
                    //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2RGBA);
                    Imgproc.cvtColor(gray, gray, Imgproc.COLOR_GRAY2RGBA);
                    bm = Bitmap.createBitmap(gray.cols(), gray.rows(), Bitmap.Config.ARGB_8888);
                    Utils.matToBitmap(gray, bm);
                    surfaceView.updateBitmap(bm);
                }
                catch (CvException e){Log.d("Exception",e.getMessage());}

            } catch (Exception e) {
                e.printStackTrace();
            }

            //long num = vuforia.getFrameQueue().element().getNumImages();

//            for (int i=0;i<num;i++) {
//                if (vuforia.getFrameQueue().element().getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
//                    rgb = vuforia.getFrameQueue().element().getImage(i);
//                }
//            }

//            Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
//            bm.copyPixelsFromBuffer(rgb.getPixels());
//
//            for (VuforiaTrackable track : relicTrackables) {
//                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) track.getListener()).getRawPose();
//
//                if (pose != null) {
//                    Matrix34F rawPose = new Matrix34F();
//                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
//                    rawPose.setData(poseData);
//
//                    Vec2F point = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(0, 0, 0));
//
//                    //Imgproc.circle();
//                }
//            }
        }

        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
        }
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

