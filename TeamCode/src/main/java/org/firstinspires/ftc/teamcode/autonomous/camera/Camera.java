package org.firstinspires.ftc.teamcode.autonomous.camera;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera{

    private static OpenCvWebcam webcam1 = null;

    public static void init(HardwareMap hardwareMap, boolean isRed) {
        //Initiates the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //Camera name from init in the Driver Station
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //Camera monitor view
        Constants.camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //Creates an object to show the camera view in the Telemetry area
        BluePipeline bluePipeline = new BluePipeline();
        RedPipeline redPipeline = new RedPipeline();
        //Sets the pipeline

        if (isRed) {
            Constants.camera.setPipeline(redPipeline);
        }else {
            Constants.camera.setPipeline(bluePipeline);
        }
        //Opening the camera
        OpenCvCamera finalCamera = Constants.camera;
        Constants.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            //live streaming
            public void onOpened() {
                finalCamera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //it will be run if the camera could not be opened
            }
        });
    }


    public static void runDetection(HardwareMap hardwareMap) {
    WebcamName webcamName =  hardwareMap.get(WebcamName.class, "Webcam 1");
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",hardwareMap.appContext.getPackageName());
    webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

    webcam1.setPipeline(new RedPipeline());
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


    }






}

