package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.utils.Vector;

public class Drivetrain {

    public final DcMotor[] dtMotors = new DcMotor[4];

    public void init(HardwareMap hardwareMap) {
        dtMotors[0] = hardwareMap.get(DcMotor.class, "lf");
        dtMotors[1] = hardwareMap.get(DcMotor.class, "lb");
        dtMotors[2] = hardwareMap.get(DcMotor.class, "rf");
        dtMotors[3] = hardwareMap.get(DcMotor.class, "rb");

        for (final  DcMotor dtMotors: dtMotors) {
            dtMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        //dtMotors[].setDirection(DcMotorSimple.Direction.REVERSE);
        //dtMotors[].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void operate(Vector velocity, double rotation) {
        final double robotAngle = Math.toRadians(Gyro.getAngle());
        drive(velocity.rotate(robotAngle), rotation);
    }

    private void drive(Vector directionNPower, double rotation) {
        final double lfPower = directionNPower.y + directionNPower.x - rotation;
        final double rfPower = directionNPower.y - directionNPower.x + rotation;
        final double lbPower = directionNPower.y - directionNPower.x - rotation;
        final double rbPower = directionNPower.y + directionNPower.x + rotation;
        double highestPower = 1;
        final double max = Math.max(1,Math.max(Math.abs(lfPower),
                Math.max(Math.abs(lbPower), Math.max(Math.abs(rfPower), Math.abs(rbPower)))));
        if (max > 1)
            highestPower = max;
        dtMotors[0].setPower(DrivetrainConstants.power * (lfPower / highestPower));
        dtMotors[1].setPower(DrivetrainConstants.power * (lbPower / highestPower));
        dtMotors[2].setPower(DrivetrainConstants.power * (rfPower / highestPower));
        dtMotors[3].setPower(DrivetrainConstants.power * (rbPower / highestPower));
    }

    public void stop() {
        for (DcMotor motor : dtMotors) {
            motor.setPower(0);
        }
    }

    public double ticksToCm(double ticks){
        return ticks * DrivetrainConstants.ticksToCM;
    }
}
