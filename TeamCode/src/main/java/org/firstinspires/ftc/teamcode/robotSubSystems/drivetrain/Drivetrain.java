package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Vector;

public class Drivetrain {

    private static final DcMotor[] dtMotors = new DcMotor[4];

    public static void init(HardwareMap hardwareMap) {
        dtMotors[0] = hardwareMap.get(DcMotor.class, "lf");
        dtMotors[1] = hardwareMap.get(DcMotor.class, "lb");
        dtMotors[2] = hardwareMap.get(DcMotor.class, "rf");
        dtMotors[3] = hardwareMap.get(DcMotor.class, "rb");

        for (final  DcMotor dtMotors: dtMotors) {
            dtMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        dtMotors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        dtMotors[3].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static void operate(Gamepad gamepad) {
        drive(gamepad);
    }

    private static void drive(Gamepad gamepad) {
        final Vector vectorGamepad = new Vector(gamepad.left_stick_x, gamepad.left_stick_y);
        vectorGamepad.rotate(Angle.wrapAngle0_360(Gyro.getAngle()));

        dtMotors[0].setPower(Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed,vectorGamepad.y + vectorGamepad.x + gamepad.right_trigger - gamepad.left_trigger)));
        dtMotors[1].setPower(Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed,vectorGamepad.y - vectorGamepad.x + gamepad.right_trigger - gamepad.left_trigger)));
        dtMotors[2].setPower(Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed,vectorGamepad.y - vectorGamepad.x - gamepad.right_trigger + gamepad.left_trigger)));
        dtMotors[3].setPower(Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed,vectorGamepad.y + vectorGamepad.x - gamepad.right_trigger + gamepad.left_trigger)));
    }

    private static PID moveRobotPID = new PID(DrivetrainConstants.moveRobotKp,DrivetrainConstants.moveRobotKi,DrivetrainConstants.moveRobotKd,DrivetrainConstants.moveRobotKf,DrivetrainConstants.moveRobotIzone,DrivetrainConstants.moveRobotMaxSpeed,DrivetrainConstants.moveRobotMinSpeed);
    private static PID turnRobotPID = new PID(DrivetrainConstants.turnRobotKp,DrivetrainConstants.turnRobotKi,DrivetrainConstants.turnRobotKd,DrivetrainConstants.turnRobotKf,DrivetrainConstants.turnRobotIzone,DrivetrainConstants.turnRobotMaxSpeed,DrivetrainConstants.turnRobotMinSpeed);
    public static void moveRobot(Pose2D wanted){
        wanted.vector.rotate(Angle.wrapAngle0_360(Gyro.getAngle()));
        final double lfRbWantedPosition = wanted.getY() + wanted.getX();
        final double lbRfWantedPosition = wanted.getY() - wanted.getX();

    }

    public static Vector getEncoderPos(){
        return new Vector(dtMotors[0].getCurrentPosition(), dtMotors[1].getCurrentPosition());//TODO: change to the right motor
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
