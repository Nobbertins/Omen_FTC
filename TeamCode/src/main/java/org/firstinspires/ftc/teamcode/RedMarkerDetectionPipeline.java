package org.firstinspires.ftc.teamcode;
//Camera lens was 12.5cm from right side of tile and at edge of tile when these values were calibrated

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class RedMarkerDetectionPipeline extends OpenCvPipeline {

    //Color to detect Defined as int 1 for red 2 for blue
    private int color = 1;


    private Mat YCbCr = new Mat();
    private Mat leftCrop;
    private Mat rightCrop;
    private Double leftavgin;
    private Double rightavgin;
    private Mat outPut = new Mat();

    //Currently crops only include the bottom third of the image as current camera position only ever sees marker in bottom
    private Rect leftRect = new Rect(1, 200, 319, 159);

    private Rect rightRect = new Rect(320, 200, 319, 159);



    private boolean sampling = true;

        /*
        position of left, right, middle are determined as the left line being the line closest to the backdrop
        */

    private MarkerPosition currentMarkerPosition = MarkerPosition.UNKNOWN;

    public MarkerPosition getCurrentMarkerPosition() {
        return currentMarkerPosition;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

        input.copyTo(outPut);

        leftCrop = YCbCr.submat(leftRect);
        rightCrop = YCbCr.submat(rightRect);

        Core.extractChannel(leftCrop, leftCrop, color);
        Core.extractChannel(rightCrop, rightCrop, color);

        Scalar leftavg = Core.mean(leftCrop);
        Scalar rightavg = Core.mean(rightCrop);

        leftavgin = leftavg.val[0];
        rightavgin = rightavg.val[0];
        //lower -> more sensitive to choosing left or right
        int sensitivity = 5;

        //when viewing just right and middle lines
        if (sampling){
            if (leftavgin - rightavgin < sensitivity && leftavgin - rightavgin > -sensitivity) {
                //nothing detected must be in furthest left lane
                currentMarkerPosition = MarkerPosition.RIGHT;
            } else if (leftavgin > rightavgin) {
                //on the left of screen which is middle line
                currentMarkerPosition = MarkerPosition.MIDDLE;
            } else {
                //on the right
                currentMarkerPosition = MarkerPosition.LEFT;
            }
        }

        return(outPut);

    }







}