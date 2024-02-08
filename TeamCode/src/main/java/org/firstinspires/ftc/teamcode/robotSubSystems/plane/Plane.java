package org.firstinspires.ftc.teamcode.robotSubSystems.plane;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Plane {

    private static Servo planeServo;

    public static void init(HardwareMap hardwareMap) {
        planeServo = hardwareMap.get(Servo.class, "planeServo");
    }

    public static void operate(boolean plane) {
        if (plane) {
            planeServo.setPosition(0);
        } else {
            planeServo.setPosition(0.5);
        }
    }
}
