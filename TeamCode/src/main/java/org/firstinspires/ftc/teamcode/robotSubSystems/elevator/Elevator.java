package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.PID;

public class Elevator {
    private static DcMotor leftMotor;
    private static DcMotor rightMotor;

    private static int wantedPosition = 0;

    private static final PID changeLevelPID = new PID(
            ElevatorConstance.changeLevelKp,
            ElevatorConstance.changeLevelKi,
            ElevatorConstance.changeLevelKd,
            ElevatorConstance.changeLevelKf,
            ElevatorConstance.changeLevelIzone,
            ElevatorConstance.changeLevelMaxSpeed,
            ElevatorConstance.changeLevelMinSpeed);


    public static void init(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotor.class, "0");
        leftMotor = hardwareMap.get(DcMotor.class, "1");
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    static double power = 0;
    public static void operate(ElevatorState wantedState, Gamepad gamepad) {
        int wantedPos = 0;
        switch (wantedState){
            case INTAKE:
                wantedPos = ElevatorConstance.intakePos;
                break;
            case LEVEL1:
                wantedPos = ElevatorConstance.level1Pos;
                break;
            case LEVEL2:
                wantedPos = ElevatorConstance.level2Pos;
                break;
            case LEVEL3:
                wantedPos = ElevatorConstance.level3Pos;
                break;
        }
        changeLevelPID.setWanted(wantedPos);

        power = changeLevelPID.update(getElevatorPos());

        leftMotor.setPower(-gamepad.right_stick_y);
        rightMotor.setPower(-gamepad.right_stick_y);

        wantedPosition = wantedPos;
    }
    private static int encoderResetVal = 0;
    private static int encoderResetValL = 0;
    public static int getElevatorPos() {
        return rightMotor.getCurrentPosition() - encoderResetVal;
    }

    public static int getWantedPos(){return wantedPosition;}

    public static double getElevatorPosL() {
//        return leftMotor.getCurrentPosition() - encoderResetValL;
        return power;
    }

    public static void resetEncoder(){
        encoderResetVal = rightMotor.getCurrentPosition();
        encoderResetValL = leftMotor.getCurrentPosition();
    }

    private static void setFloor(int wantedPos){

        changeLevelPID.setWanted(wantedPos);

        leftMotor.setPower(changeLevelPID.update(leftMotor.getCurrentPosition()));
        rightMotor.setPower(changeLevelPID.update(leftMotor.getCurrentPosition()));
    }

}
