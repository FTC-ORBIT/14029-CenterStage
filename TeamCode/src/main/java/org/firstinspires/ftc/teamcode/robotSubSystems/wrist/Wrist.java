package org.firstinspires.ftc.teamcode.robotSubSystems.wrist;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    private static Servo rightServo;
    private static Servo leftServo;

    public static void init(HardwareMap hardwareMap){
        rightServo = hardwareMap.get(Servo.class, "");


    }

}
