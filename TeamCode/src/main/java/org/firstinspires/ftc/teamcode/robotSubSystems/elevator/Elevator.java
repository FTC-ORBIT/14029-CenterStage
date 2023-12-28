package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.PID;

public class Elevator {
    private static DcMotor leftMotor;
    private static DcMotor rightMotor;

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
    }
    public static void operate(ElevatorState wantedLevel) {

    }
    private static int encoderResetVal = 0;
    public static int getElevatorPos() {
        return rightMotor.getCurrentPosition() - encoderResetVal;
    }

    public static int getElevatorPosL() {
        return leftMotor.getCurrentPosition() ;
    }

    public static void resetEncoder(){
        encoderResetVal = rightMotor.getCurrentPosition();
    }

    private static void setFloor(int wantedPos){

        changeLevelPID.setWanted(wantedPos);

        leftMotor.setPower(changeLevelPID.update(leftMotor.getCurrentPosition()));
        rightMotor.setPower(changeLevelPID.update(leftMotor.getCurrentPosition()));
    }

}
