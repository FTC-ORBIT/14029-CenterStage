package org.firstinspires.ftc.teamcode.robotSubSystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private static DcMotor upperBar;
    private static DcMotor sideWheels;
    public static void init(HardwareMap hardwareMap){
        upperBar = hardwareMap.get(DcMotor.class, "3");
        sideWheels = hardwareMap.get(DcMotor.class, "2");

        upperBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sideWheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sideWheels.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static void operate(IntakeState state){

        switch (state){
            case INTAKE:
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
                break;
        }

    }


}
