
package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Point;
import android.util.AttributeSet;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import static android.R.attr.height;
import static android.R.attr.width;

/**
 * Created by justin on 5/30/17.
 */

public class MySurfaceView extends SurfaceView implements SurfaceHolder.Callback{
    Bitmap bitmap;
    Point[] points;
    SurfaceHolder holder;
    Canvas canvas;

    public MySurfaceView(Context context, int width, int height) {
        super(context);
        bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        getHolder().addCallback(this);
    }

    public MySurfaceView(Context context) {
        super(context);
        bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        getHolder().addCallback(this);
    }

    public MySurfaceView(Context context, AttributeSet attrs) {
        super(context, attrs);
        getHolder().addCallback(this);

    }

    //this is called when the application has opened, and the surface has been created
    //so we start all our graphics tools at this stage
    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        this.holder = holder;
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {}

    //this method runs when the surface is destroyed, so we kill the graphics thread
    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {}


    //these methods are for passing graphics information from Sandbox to be drawn here
    public void updateBitmap(Bitmap b)
    {
        bitmap=b;
        canvas = holder.lockCanvas();
        Paint paint = new Paint();
        canvas.rotate(90);
        canvas.drawBitmap(bitmap, 0,-720, paint);
        holder.unlockCanvasAndPost(canvas);

    }
    public void updatePoints(Point[] p){
        points=p;
    }

}