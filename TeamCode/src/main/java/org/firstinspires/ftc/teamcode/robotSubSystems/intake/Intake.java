package org.firstinspires.ftc.teamcode.robotSubSystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {

    private static DcMotor upperBar;
    private static DcMotor sideWheels;
    private static Servo intakeServo;
    public static DistanceSensor distanceSensor;
    public static void init(HardwareMap hardwareMap){
        upperBar = hardwareMap.get(DcMotor.class, "3");
        sideWheels = hardwareMap.get(DcMotor.class, "2");
        intakeServo = hardwareMap.get(Servo.class, "iS");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        upperBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sideWheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sideWheels.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static void servoTest(Gamepad gamepad) {
//        if(touchSensor.isPressed()) {intakeServo.setPosition(IntakeConstance.intakeServoOpenPos);}
        if (gamepad.a) {intakeServo.setPosition(IntakeConstance.intakeServoMiddlePos);}
        if (gamepad.b) {intakeServo.setPosition(IntakeConstance.intakeServoClosedPos);}
    }

    private static boolean firstTimeInIntake = true;
    private static boolean firstTimePressed = false;
    private static double startPressedTime = 0;
    private static double startTime = 0;


    private static final ElapsedTime timer = new ElapsedTime();

    private static IntakeState lastState = IntakeState.INTAKE;
    public static void operate(IntakeState state){
        firstTimeInIntake = lastState != state;
        switch (state){
            case INTAKE:
                if (firstTimeInIntake){
                    intakeServo.setPosition(IntakeConstance.intakeServoMiddlePos);
                    firstTimePressed = true;
                    firstTimeInIntake = false;
                    startTime = timer.milliseconds();
                }
                if(distanceSensor.getDistance(DistanceUnit.MM) < 33 && firstTimePressed && timer.milliseconds() - startTime > 400) {
                    startPressedTime = timer.milliseconds();
                    firstTimePressed = false;
                }
                if (firstTimePressed){
                    startPressedTime = timer.milliseconds();}
                if (timer.milliseconds() - startPressedTime > 500){
                    intakeServo.setPosition(IntakeConstance.intakeServoOpenPos);
                }
                upperBar.setPower(1);
                sideWheels.setPower(1);
                break;
            case DEPLETE:
                upperBar.setPower(-0.4);
                sideWheels.setPower(-0.4);
                break;
            case STOP:
                upperBar.setPower(0);
                sideWheels.setPower(0);
                intakeServo.setPosition(IntakeConstance.intakeServoClosedPos);
                break;
        }
        lastState = state;

    }


}
