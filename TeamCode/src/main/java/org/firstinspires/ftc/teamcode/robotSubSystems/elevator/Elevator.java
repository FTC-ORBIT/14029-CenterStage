package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.PID;

public class Elevator {
    private static DcMotor leftMotor;
    private static DcMotor rightMotor;

    private static int wantedPos = ElevatorConstance.intakePos;

    private static final PID changeLevelPID = new PID(
            ElevatorConstance.changeLevelKp,
            ElevatorConstance.changeLevelKi,
            ElevatorConstance.changeLevelKd,
            ElevatorConstance.changeLevelKf,
            ElevatorConstance.changeLevelIzone,
            ElevatorConstance.changeLevelMaxSpeed,
            ElevatorConstance.changeLevelMinSpeed);


    public static void init(HardwareMap hardwareMap) {
        rightMotor = hardwareMap.get(DcMotor.class, "1");
        leftMotor = hardwareMap.get(DcMotor.class, "0");
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private static double power = 0;
    private static ElevatorState lastWantedState = ElevatorState.INTAKE;
    public static void operate(ElevatorState wantedState, double gamepadVal) {
        if (gamepadVal == 0 && wantedState != lastWantedState) {
            switch (wantedState) {
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
                case MOVE:
                    wantedPos = 0;
            }
            lastWantedState = wantedState;
        }
        if (wantedState != ElevatorState.INTAKE) {
            wantedPos -= (int) gamepadVal * 50;
        }

        changeLevelPID.setWanted(wantedPos);

        power = changeLevelPID.update(getElevatorPos());
        leftMotor.setPower(power);
        rightMotor.setPower(power);

    }
    private static int encoderResetVal = 0;
    private static int encoderResetValL = 0;
    public static int getElevatorPos() {
        return leftMotor.getCurrentPosition() - encoderResetVal;
    }

    public static int getWantedPos(){return wantedPos;}

    public static double getElevatorPosL() {
        return leftMotor.getCurrentPosition() - encoderResetValL;
//        return power;
    }

    public static void resetEncoder(){
        encoderResetVal = leftMotor.getCurrentPosition();
        encoderResetValL = leftMotor.getCurrentPosition();
    }

    private static void setFloor(int wantedPos){

        changeLevelPID.setWanted(wantedPos);

        leftMotor.setPower(changeLevelPID.update(leftMotor.getCurrentPosition()));
        rightMotor.setPower(changeLevelPID.update(leftMotor.getCurrentPosition()));
    }

}
