package org.firstinspires.ftc.teamcode.Supers;

import android.app.Activity;
import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.support.annotation.Nullable;
import android.util.Log;
import android.view.View;
import android.widget.LinearLayout;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.MySurfaceView;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

import java.util.Locale;

/**
 * Created by moham on 1/15/18.
 */
public class Robot extends OpMode {


    public VuforiaLocalizer vuforia;
    public int cameraMonitorViewId;
    public VuforiaTrackables relicTrackables;
    public VuforiaTrackable relicTemplate;
    public VuforiaTrackableDefaultListener listener;
    public RelicRecoveryVuMark vuMark;
    public Image rgb;
    public Context context;
    public static int WIDTH=1280, HEIGHT=720;
    public MySurfaceView surfaceView;

    Handler handler;

    public DcMotorEx leftBack, leftFront, rightFront, rightBack, slideLeft, slideRight, intakeRight, intakeLeft;
    public Servo flipLeft, flipRight, relicFlipper, relicGrabber, jewelArm, jewelTurn;
    public CRServo relic1, relic2, relic3, relic4;

    public BNO055IMU imu;

    public Orientation angles;
    Acceleration gravity;

    public BNO055IMU.Parameters imuparameters;

    public boolean resetPosition = true;
    public boolean reset = false;
    public boolean resetPositionJewel = true;
    public static final int ENCODER_TICKS_PER_CM = 17 ; //  538/(10 * Math.PI) = 17.1337579618
    public int targetTicks;
    public double PID_P = .003;
    public double flipRightUp = .95;
    public double flipRightDown = .3;
    public double flipRightMiddle = .45;
    public double flipLeftUp = .05;
    public double flipLeftDown = .7 ;
    public double flipLeftMiddle = .55 ;
    public double jewelArmUp = .275;
    public double jewelArmIn = .01;
    public double jewelArmDown = .99;
    public double jewelTurnCenter = .24;
    public double jewelTurnFront = .05;
    public double jewelTurnBack = .45;
    public double relicFlipperDown = .225;
    public double relicFlipperMid = .53;
    public double relicFlipperUp = .8;
    public double relicGrabberIn = .6;
    public double relicGrabberOut = .1;
    public double relicGrabberCurrent;

    public boolean relicGrabberOuting = false;
    public int relicGrabberPosition = 0;

    public String jewelColor = "BLUE";
    public String columnKey = "LEFT";
    public boolean detected = false;
    public boolean gyroInitialized = false;
    public double startAngle;

    public boolean isDriving = false;

    public enum RelicGrabberState {
        In, Out
    }

    public enum RelicFlipperState {
        Down, Middle, Up
    }


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
        jewelArm = hardwareMap.servo.get("jewelArm");
        jewelTurn = hardwareMap.servo.get("jewelTurn");
        relicFlipper = hardwareMap.servo.get("relicFlipper");
        relicGrabber = hardwareMap.servo.get("relicGrabber");

        relic1 = hardwareMap.crservo.get("relic1");
        relic2 = hardwareMap.crservo.get("relic2");
        relic3 = hardwareMap.crservo.get("relic3");
        relic4 = hardwareMap.crservo.get("relic4");

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

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flipLeft.setPosition(flipLeftDown);
        flipRight.setPosition(flipRightDown);
        jewelArm.setPosition(jewelArmIn);
        jewelTurn.setPosition(jewelTurnCenter);
        relicFlipper.setPosition(relicFlipperDown);
        relicGrabber.setPosition(relicGrabberIn);
        relicGrabberCurrent = relicGrabber.getPosition();
    }

    @Override
    public void loop() {

    }

    public void initAutonomous() {
//        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        imuparameters = new BNO055IMU.Parameters();
        imuparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuparameters.loggingEnabled      = true;
        imuparameters.loggingTag          = "IMU";
        imuparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuparameters);

        // Set up our telemetry dashboard
        composeTelemetry();

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

    public void stopAutonomous() {
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

    // ------------------------------------------------------------------------------------//
    //--------------------------Autonomous Methods-----------------------------------------//
    // ------------------------------------------------------------------------------------//

    public boolean driveForEncoder(DcMotor motor, int centimeters) {
        if (resetPosition) {
                resetPosition=false;
                targetTicks = motor.getCurrentPosition() + (centimeters * ENCODER_TICKS_PER_CM);
            }
            if (Math.abs(targetTicks - motor.getCurrentPosition()) > 5) {
                motor.setPower((targetTicks - motor.getCurrentPosition()) * PID_P);
                return false;
            } else {
                motor.setPower(0);
                resetPosition = true;
                return true;
        }
    }

//----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
