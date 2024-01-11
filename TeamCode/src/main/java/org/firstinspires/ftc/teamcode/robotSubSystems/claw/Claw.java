package org.firstinspires.ftc.teamcode.robotSubSystems.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private static Servo leftServo;
    private static Servo rightServo;

    public static void init(HardwareMap hardwareMap){
        leftServo = hardwareMap.get(Servo.class, "claw left");
        rightServo = hardwareMap.get(Servo.class, "claw right");

    }

    public static void operate(ClawState state){
        switch (state){
            case OPEN:
                leftServo.setPosition(ClawConstance.openLeftPos);
                rightServo.setPosition(ClawConstance.openRightPos);
                break;
            case CLOSED:
                leftServo.setPosition(ClawConstance.closedLeftPos);
                rightServo.setPosition(ClawConstance.closedRightPos);
                break;
        }

    }

}
