package org.firstinspires.ftc.teamcode.autonomous.camera;


import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;

public final class Constants {
    //HSV constants
    public static Scalar redLowHSV = new Scalar(0 , 125 , 35);
    public static Scalar redHighHSV = new Scalar(90 , 255 , 255);

    public static Scalar blueLowHSV = new Scalar(107 , 128,78);
    public static Scalar blueHighHSV = new Scalar(117 , 217 , 143);
    //Scalar (color) constants
    public static Scalar Green = new Scalar(0 , 255 ,0);
    public static Scalar White = new Scalar(255 , 255 , 255);
    public static Scalar Red = new Scalar(255 , 0 , 0);
    public static int binary = Imgproc.COLOR_RGB2HSV;
    //Blur size radius
    public static Size BlurRadius = new Size(37,37);
    //camera
    public static OpenCvCamera camera;
}