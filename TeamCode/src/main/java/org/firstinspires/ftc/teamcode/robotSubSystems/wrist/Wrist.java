package org.firstinspires.ftc.teamcode.robotSubSystems.wrist;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    private static Servo rightServo;
    private static Servo leftServo;

    public static void init(HardwareMap hardwareMap){
        rightServo = hardwareMap.get(Servo.class, "right servo");
        leftServo = hardwareMap.get(Servo.class, "left servo");

    }

    public static void operate(WristState state){
        switch (state){
            case DEPLETE:
                rightServo.setPosition(WristConstance.rightServoDepletePos);
                leftServo.setPosition(WristConstance.leftServoDepletePos);
                break;
            case INTAKE:
                rightServo.setPosition(WristConstance.rightServoIntakePos);
                leftServo.setPosition(WristConstance.leftServoIntakePos);
                break;
        }
    }

}
