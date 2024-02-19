package org.firstinspires.ftc.teamcode.robotSubSystems.wrist;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wrist {
    private static Servo rightServo;
    public static Servo leftServo;

//    private static TouchSensor touchSensor;

    public static void init(HardwareMap hardwareMap){
        rightServo = hardwareMap.get(Servo.class, "wrist right");
        leftServo = hardwareMap.get(Servo.class, "wrist left");

//        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        rightServo.resetDeviceConfigurationForOpMode();
        leftServo.resetDeviceConfigurationForOpMode();

    }

    public static void operate(WristState state){
//        if (touchSensor.isPressed()) {state = WristState.DEPLETE;}
        switch (state){
            case DEPLETE:
                rightServo.setPosition(WristConstance.rightServoDepletePos);
                leftServo.setPosition(WristConstance.leftServoDepletePos);
                break;
            case INTAKE:
                rightServo.setPosition(WristConstance.rightServoIntakePos);
                leftServo.setPosition(WristConstance.leftServoIntakePos);
                break;
            case GROUND:
                rightServo.setPosition(WristConstance.rightServoGroundPos);
                leftServo.setPosition(WristConstance.leftServoGroundPos);
                break;
        }
    }

}
