package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    public DcMotor elevMotor;

    public void init(HardwareMap hardwareMap) {
        elevMotor = hardwareMap.get(DcMotor.class, "elevMotor");
        elevMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void operate(double power) {
        elevMotor.setPower(power);
    }

    public int getMotorPos() {
        return elevMotor.getCurrentPosition();
    }

}
