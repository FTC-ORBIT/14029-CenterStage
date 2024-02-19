package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstance;
import org.firstinspires.ftc.teamcode.robotSubSystems.poseTracker.PoseTracker;
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

        for (final DcMotor dtMotors : dtMotors) {
            dtMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        dtMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        dtMotors[3].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static void operate(Vector vector, double rotation) {
        drive(
                slowedVec(
                        fieldCentric(vector),
                        Elevator.getElevatorPos(),
                        ElevatorConstance.level3Pos,
                        0.40
                ),
                rotation
        );
    }

    private static Vector slowedVec(Vector vector, double current, double highest, double speed) {
        //0.32
        return vector.scale(Math.max(speed, (highest - current) / highest));
    }

    private static Vector fieldCentric(Vector gamepad) {
        gamepad = gamepad.rotate(-Math.toRadians(Angle.wrapAngle0_360(Gyro.getAngle())));
        return gamepad;
    }

    private static void drive(Vector vector, double rotation) {
        dtMotors[0].setPower(Math.signum(vector.y + vector.x + rotation) * Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed, Math.abs(vector.y + vector.x + rotation))));
        dtMotors[1].setPower(Math.signum(vector.y - vector.x + rotation) * Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed, Math.abs(vector.y - vector.x + rotation))));
        dtMotors[2].setPower(Math.signum(vector.y - vector.x - rotation) * Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed, Math.abs(vector.y - vector.x - rotation))));
        dtMotors[3].setPower(Math.signum(vector.y + vector.x - rotation) * Math.max(DrivetrainConstants.minDriveSpeed, Math.min(DrivetrainConstants.maxDriveSpeed, Math.abs(vector.y + vector.x - rotation))));
    }

    private static PID moveRobotPID = new PID(DrivetrainConstants.moveRobotKp, DrivetrainConstants.moveRobotKi, DrivetrainConstants.moveRobotKd, DrivetrainConstants.moveRobotKf, DrivetrainConstants.moveRobotIzone, DrivetrainConstants.moveRobotMaxSpeed, DrivetrainConstants.moveRobotMinSpeed);

    private static PID turnRobotPID = new PID(DrivetrainConstants.turnRobotKp, DrivetrainConstants.turnRobotKi, DrivetrainConstants.turnRobotKd, DrivetrainConstants.turnRobotKf, DrivetrainConstants.turnRobotIzone, DrivetrainConstants.turnRobotMaxSpeed, DrivetrainConstants.turnRobotMinSpeed);
    public static boolean isFinished = false;
    private static boolean firstTime = true;

    private static double lfPower = 0;
    private static double rfPower = 0;

    public static void moveRobot(Pose2D wanted , Telemetry telemetry) {
        wanted.vector.rotate(Angle.wrapAngle0_360(Gyro.getAngle()));


        final double lfWanted = wanted.getY() + wanted.getX();
        final double rfWanted = wanted.getY() - wanted.getX();


        final double lfCurrent = PoseTracker.getPose().getY() + PoseTracker.getPose().getX();
        final double rfCurrent = PoseTracker.getPose().getY() - PoseTracker.getPose().getX();



        final double lfError = lfWanted - lfCurrent;
        final double rfError = rfWanted - rfCurrent;

            if (lfError > rfError || true) {
                moveRobotPID.setWanted(lfWanted);
                lfPower = moveRobotPID.update(lfCurrent);
                rfPower = (rfError / lfError) * lfPower;
            } else {
                moveRobotPID.setWanted(rfWanted);
                rfPower = moveRobotPID.update(rfCurrent);
                lfPower = (lfError / rfError) * rfPower;
            }



//        turn robot pid

        turnRobotPID.setWanted(wanted.getAngle());

        double rotationPower = turnRobotPID.update(PoseTracker.getPose().getAngle());

        telemetry.addData("power", lfPower);

        dtMotors[0].setPower(lfPower + rotationPower);
        dtMotors[1].setPower(rfPower - rotationPower);
        dtMotors[2].setPower(rfPower + rotationPower);
        dtMotors[3].setPower(lfPower - rotationPower);

        final boolean isFinishedLf = Math.abs(lfWanted) - 300 < Math.abs(lfCurrent) && Math.abs(lfCurrent) < Math.abs(lfWanted) + 300;
        final boolean isFinishedRf = Math.abs(rfWanted) - 300 < Math.abs(rfCurrent) && Math.abs(rfCurrent) < Math.abs(rfWanted) + 300;
        final boolean isFinishedTurning = Math.abs(wanted.getAngle()) - 30 < Math.abs(PoseTracker.getPose().getAngle()) && Math.abs(PoseTracker.getPose().getAngle()) < Math.abs(wanted.getAngle()) + 30;

        isFinished = isFinishedLf && isFinishedRf && isFinishedTurning;
        if(isFinished){
            breakMotors();
        }
    }



    public static Vector getEncoderPos() {
        return new Vector(0, dtMotors[3].getCurrentPosition());//TODO: change to the right motor
    }

    public static void resetEncoders(){
        dtMotors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dtMotors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    private static void breakMotors() {
        for (DcMotor motor : dtMotors) {
            motor.setPower(0);
        }
    }



    public double ticksToCm(double ticks) {
        return ticks * DrivetrainConstants.ticksToCM;
    }
}
