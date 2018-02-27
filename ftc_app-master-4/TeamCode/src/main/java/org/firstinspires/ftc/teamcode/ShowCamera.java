package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.support.annotation.Nullable;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
/**
 * Created by moham on 11/4/17.
 */

@TeleOp
public class ShowCamera extends OpMode {
    VuforiaLocalizer localizer;
    VuforiaTrackableDefaultListener wheels;
    float mmPerInch = 25.4f;

    public void init() {
        // can uncomment then comment out other parameters to make it show vuforia on screen
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "Abn65kT/////AAAAGYXh41emd0DbrJk0zJJRufqDtNESV/FU6MKggz5cCklQ0ciEFHgTjNhGXLqicZY52rE/S4b6cqaFlBOD6UoedC14/ecLwNyh1wlneiSWcLVyitBkHhgWeQQFUtJmAGVEoFzsrr7YREaw4VH2DPHbn3L8xjHjc7v7fnFwTXyE19SX57dWVdEgyqoKcQ82kACSokwoZBvSXU374Afi4+/and+Xx1vFthqIBpJxqMqOeYgGCDAi1CfIXTr+OqMUJDndbrhN6y0CXn9H8MROMB7I16mxU1Q4OkB7z9q25DiUFmbh5wC7AgYd8ILLLrfCCfwsfn22KVD7JvWwMhn61o38y5X82XE4qiwx7wqjb3lvQvjp";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        localizer = ClassFactory.createVuforiaLocalizer(parameters);

        // Used to keep a queue of images that can be read from OpenCV or RenderScript
        localizer.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        VuforiaTrackables beacons = localizer.loadTrackablesFromAsset("FTC_2016-17");
        wheels = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();

        beacons.activate();
    }

    public void loop() {

//        OpenGLMatrix pose = wheels.getPose();
//        if (pose != null) {
//            Orientation orientation = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, Orientation.AngleSet.THEONE);
//            VectorF translation = pose.getTranslation(); // Column 3
//            telemetry.addData("first", translation.get(0) / mmPerInch);
//            telemetry.addData("second", translation.get(1) / mmPerInch);
//            telemetry.addData("third", translation.get(2) / mmPerInch);
//            telemetry.addData("x", orientation.firstAngle);
//            telemetry.addData("y", orientation.secondAngle);
//            telemetry.addData("z", orientation.thirdAngle);
//            telemetry.addData("size", translation.length());
//            telemetry.update();
//        }
    }

}
