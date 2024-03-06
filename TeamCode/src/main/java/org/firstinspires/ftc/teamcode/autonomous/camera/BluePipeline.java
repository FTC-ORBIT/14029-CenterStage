package org.firstinspires.ftc.teamcode.autonomous.camera;


import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BluePipeline extends OpenCvPipeline {
    //Creates a static Mat variable
    private static Mat material;
    private static Point center;
    public static ElementPosition elementPosition = ElementPosition.MIDDLE;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Constants.binary);
        Imgproc.blur(input, input, Constants.BlurRadius);
        center = Contours.getCenter(
                Contours.contourPolyList(
                        Contours.getBiggestContour(
                                Contours.getContour(input, Constants.blueLowHSV, Constants.blueHighHSV)
                        )
                )
                , input);
        if(center == null) {
            return input;
        }
        if (433 < center.x && center.x < 866) {
            elementPosition = ElementPosition.MIDDLE;
        } else if (center.x < 433) {
            elementPosition = ElementPosition.LEFT;
        } else {
            elementPosition = ElementPosition.RIGHT;
        }
        return input;
    }
    //Cloning the material (input)
    public static Mat getClonedMat() {return material.clone();}

    public static ElementPosition getElementPos(){
        return elementPosition;
    }
    //Creates a public function so we can use the material (input) in every place needed
    public static Mat getMat() {return material;}
}
