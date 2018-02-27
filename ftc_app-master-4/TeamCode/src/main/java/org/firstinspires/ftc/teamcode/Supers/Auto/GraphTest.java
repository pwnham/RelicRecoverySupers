package org.firstinspires.ftc.teamcode.Supers.Auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/**
 * Created by moham on 1/20/18.
 */
@Autonomous
public class GraphTest extends OpMode {

    private double x = 0, y = 0;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        x ++;
        y ++;
        Log.d("GraphTest", String.format("%f;%f", x, y));
    }

    @Override
    public void stop() {
    }
}
