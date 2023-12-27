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


    public void init(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "1");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor = hardwareMap.get(DcMotor.class, "0");
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void operate(ElevatorState wantedLevel) {

    }

    public int getElevatorPos() {
        return leftMotor.getCurrentPosition();
    }

    private static void setFloor(int wantedPos){

        changeLevelPID.setWanted(wantedPos);

        leftMotor.setPower(changeLevelPID.update(leftMotor.getCurrentPosition()));
        rightMotor.setPower(changeLevelPID.update(leftMotor.getCurrentPosition()));
    }

}
