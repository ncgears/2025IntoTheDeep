package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class tseSaturationProcessor implements VisionProcessor {
    //since the camera is upside down, right is left and left is right
    //this is more efficient than rotating the camera orientation which has a lot of overhead
    public Rect rectRight = new Rect(10,70,150,230);
    public Rect rectMiddle = new Rect(190, 120, 340, 180);
    public Rect rectLeft = new Rect(610, 100, 170, 200);
    Selected selection = Selected.NONE;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        double satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
        double satRectRight = getAvgSaturation(hsvMat, rectRight);

        if ((satRectLeft > satRectMiddle) && (satRectLeft > satRectRight)) {
            return Selected.LEFT;
        } else if ((satRectMiddle > satRectLeft) && (satRectMiddle > satRectRight)) {
            return Selected.MIDDLE;
        }
        return Selected.RIGHT;
    }

    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.GREEN);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.RED);

        android.graphics.Rect drawRectLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        selection = (Selected) userContext;
        switch (selection) {
            case LEFT:
                canvas.drawRect(drawRectLeft, selectedPaint);
                canvas.drawRect(drawRectMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectRight, nonSelectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRectLeft, nonSelectedPaint);
                canvas.drawRect(drawRectMiddle, selectedPaint);
                canvas.drawRect(drawRectRight, nonSelectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectLeft, nonSelectedPaint);
                canvas.drawRect(drawRectMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectRight, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectLeft, nonSelectedPaint);
                canvas.drawRect(drawRectMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectRight, nonSelectedPaint);
                break;
        }
    }

    public Selected getSelection() {
        return selection;
    }

    public enum Selected { NONE, LEFT, MIDDLE, RIGHT }
}
