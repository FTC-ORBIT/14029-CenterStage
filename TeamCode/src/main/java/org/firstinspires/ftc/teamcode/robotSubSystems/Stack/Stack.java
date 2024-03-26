package org.firstinspires.ftc.teamcode.robotSubSystems.Stack;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Stack {
    private static Servo stackServo;
    public static void init(HardwareMap hardwareMap) {
        stackServo = hardwareMap.get(Servo.class, "StackServo");
    }
    public static void operate(StackState state){
        switch (state){
            case UP:
                stackServo.setPosition(StackConstants.up);
                break;
            case FULL:
                stackServo.setPosition(StackConstants.full);
                break;
            case HALF:
                stackServo.setPosition(StackConstants.half);
                break;
            case BACKBOARD:
                stackServo.setPosition(StackConstants.backBoard);
                break;
        }
    }
}
