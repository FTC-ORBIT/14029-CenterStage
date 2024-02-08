package org.firstinspires.ftc.teamcode.robotSubSystems.intake;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {

    private static DcMotor upperBar;
    private static DcMotor sideWheels;
    private static Servo intakeServo;
    private static DistanceSensor distanceSensor;
    private static TouchSensor touchSensor;
    public static void init(HardwareMap hardwareMap){
        upperBar = hardwareMap.get(DcMotor.class, "3");
        sideWheels = hardwareMap.get(DcMotor.class, "2");
        intakeServo = hardwareMap.get(Servo.class, "iS");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
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
                }
                if(touchSensor.isPressed() && firstTimePressed || distanceSensor.getDistance(DistanceUnit.CM) < 2 && firstTimePressed) {
                    startTime = timer.milliseconds();
                    firstTimePressed = false;
                }
                if (firstTimePressed){startTime = timer.milliseconds();}
                if (timer.milliseconds() - startTime > 1000){
                    intakeServo.setPosition(IntakeConstance.intakeServoOpenPos);
                }
                upperBar.setPower(1);
                sideWheels.setPower(1);
                break;
            case DEPLETE:
                upperBar.setPower(-0.6);
                sideWheels.setPower(-0.6);
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
