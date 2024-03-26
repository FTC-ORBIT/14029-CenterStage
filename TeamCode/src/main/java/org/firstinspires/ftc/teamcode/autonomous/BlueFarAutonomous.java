package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.autonomous.camera.BluePipeline;
import org.firstinspires.ftc.teamcode.autonomous.camera.Camera;
import org.firstinspires.ftc.teamcode.autonomous.camera.ElementPosition;
import org.firstinspires.ftc.teamcode.autonomous.camera.RedPipeline;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstance;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorState;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.poseTracker.PoseTracker;
import org.firstinspires.ftc.teamcode.robotSubSystems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.robotSubSystems.wrist.WristState;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.utils.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Vector;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "blue far")
public class BlueFarAutonomous extends LinearOpMode {

    private static int actionNum = 1;
    private static final ElapsedTime timer = new ElapsedTime();
    private static RobotState state = RobotState.CLIMB;
    private static RobotState lastState = RobotState.TRAVEL;


    ElevatorState elevatorState = ElevatorState.INTAKE;
    IntakeState intakeState = IntakeState.STOP;
    ClawState clawState = ClawState.CLOSED;
    WristState wristState = WristState.INTAKE;

    private static double stopIntakeStartTime = 0;
    private static double startIntakeStartTime = 0;
    private static boolean firstTimeInIntake = true;

    private static boolean firstTimeInTravel = true;

    ElevatorState lastElevatorState = ElevatorState.INTAKE;

    ElementPosition elementPos = ElementPosition.LEFT;


    public void initRobot() {
        Gyro.init(hardwareMap);
        Drivetrain.init(hardwareMap);
        Elevator.init(hardwareMap);
        Intake.init(hardwareMap);
        Wrist.init(hardwareMap);
        Claw.init(hardwareMap);
//        AprilTagDetector.runAprilTagDetection(this);
        Drivetrain.resetEncoders();
        PoseTracker.resetPos();
        Camera.init(hardwareMap, false);
        hasStarted = true;
        state = RobotState.CLIMB;
        while (opModeInInit()) {
            telemetry.addData("element pos", BluePipeline.getElementPos());
            telemetry.update();
        }
    }

    public void operate() {
        actionNum = gamepad1.a ? 1 : actionNum;
        state = gamepad1.a ? RobotState.CLIMB : state;
        if (state == RobotState.CLIMB) {
            state = RobotState.TRAVEL;
            elementPos = BluePipeline.getElementPos();
            actionNum = 1;
        }

        switch (elementPos) {
            case LEFT:
                switch (actionNum) {
                    case 1:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 27000), 0), telemetry);
                        telemetry.addData("posY", PoseTracker.getPose().getY());
                        telemetry.addData("posX", PoseTracker.getPose().getX());
                        telemetry.addData("angle", PoseTracker.getPose().getAngle());
                        break;
                    case 2:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0), -45), telemetry);
                        break;
                    case 3:
                        state = RobotState.DEPLETE;
                        waitAuto(1000);
                        break;
                    case 4:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 11000), -45), telemetry);
                        intakeState = IntakeState.DEPLETE;
                        state = RobotState.TRAVEL;
                        break;
                    case 5:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, -25000), -45), telemetry);
                        break;
                    case 6:
                        intakeState = IntakeState.STOP;
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0), 90), telemetry);
                        break;
                    case 7:
                        Drivetrain.moveRobot(new Pose2D(new Vector(-50000, 0), 90), telemetry);
                        state = RobotState.TRAVEL;
                        break;
                    case 8:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0), 90), telemetry);
                        break;
                    case 9:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, -120000), 90), telemetry);
                        break;
                    case 10:
                        Drivetrain.moveRobot(new Pose2D(new Vector(46000, 0), 90), telemetry);
                        elevatorState = ElevatorState.LEVEL1;
                        break;
                    case 11:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0), 90), telemetry);
                        break;
                    case 12:
                        Drivetrain.driveByTime(0.2);
                        waitAuto(2500);
                        break;
                    case 13:
                        waitAuto(500);
                        state = RobotState.DROP;
                        break;
                    case 14:
                        waitAuto(500);
                        elevatorState = ElevatorState.LEVEL1;
                        break;
                    case 15:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 8000), 90), telemetry);
                        break;
                    case 16:
                        waitAuto(2000);
                        state = RobotState.INTAKE;
                        break;
                    case 17:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0), 0), telemetry);
                        break;
                }
                break;
            case MIDDLE:
                switch (actionNum) {
                    case 1:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 29000), 0), telemetry);
                        state = RobotState.TRAVEL;
                        telemetry.addData("posY", PoseTracker.getPose().getY());
                        telemetry.addData("posX", PoseTracker.getPose().getX());
                        telemetry.addData("angle", PoseTracker.getPose().getAngle());
                        break;
                    case 2:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0), -150), telemetry);
                        break;
                    case 3:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, -31000), -150), telemetry);
                        state = RobotState.TRAVEL;
                        break;
                    case 4:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0), -110), telemetry);
                        break;
                    case 5:
                        state = RobotState.DEPLETE;
                        waitAuto(1000);
                        break;
                    case 6:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 10500), -110), telemetry);
                        state = RobotState.TRAVEL;
                        intakeState = IntakeState.DEPLETE;
                        break;
                    case 7:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, -15000), -110), telemetry);
                        break;
                    case 8:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0), 88), telemetry);
                        intakeState = IntakeState.STOP;
                        break;
                    case 9:
                        Drivetrain.moveRobot(new Pose2D(new Vector(-7000, 0), 88), telemetry);
                        break;
                    case 10:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0), 88), telemetry);
                        break;
                    case 11:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, -130000), 88), telemetry);
                        break;
                    case 12:
                        Drivetrain.moveRobot(new Pose2D(new Vector(32000, 0), 88), telemetry);
                        elevatorState = ElevatorState.LEVEL1;
                        break;
                    case 13:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0), 88), telemetry);
                        break;
                    case 14:
                        waitAuto(3000);
                        Drivetrain.driveByTime(0.2);
                        break;
                    case 15:
                        waitAuto(1500);
                        state = RobotState.DROP;
                        break;
                    case 16:
                        waitAuto(500);
                        elevatorState = ElevatorState.LEVEL2;
                        break;
                    case 17:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 8000), 90), telemetry);
                        break;
                    case 18:
                        state = RobotState.INTAKE;
                        waitAuto(1000);
                        break;
                    case 19:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0), 0), telemetry);
                        break;
                }
                break;
            case RIGHT:
                switch (actionNum) {
                    case 1:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 34000), 0), telemetry);
                        break;
                    case 2:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0), 90), telemetry);
                        break;
                    case 3:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, -3000), 90), telemetry);
                        break;
                    case 4:
                        state = RobotState.DEPLETE;
                        waitAuto(1000);
                        break;
                    case 5:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 7000), 90), telemetry);
                        state = RobotState.TRAVEL;
                        intakeState = IntakeState.DEPLETE;
                        break;
                    case 6:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, -2000), 90), telemetry);
                        break;
                    case 7:
                        Drivetrain.moveRobot(new Pose2D(new Vector(-30000, 0), 90), telemetry);
                        intakeState = IntakeState.STOP;
                        state = RobotState.TRAVEL;
                        break;
                    case 8:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0), 90), telemetry);
                        state = RobotState.TRAVEL;
                        break;
                    case 9:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, -110000), 90), telemetry);
                        break;
                    case 10:
                        Drivetrain.moveRobot(new Pose2D(new Vector(25000, 0), 90), telemetry);
                        elevatorState = ElevatorState.LEVEL1;
                        break;
                    case 11:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0), 90), telemetry);
                        break;
                    case 12:
                        waitAuto(2500);
                        Drivetrain.driveByTime(0.2);
                        break;
                    case 13:
                        state = RobotState.DROP;
                        waitAuto(500);
                        break;
                    case 14:
                        elevatorState = ElevatorState.LEVEL1;
                        waitAuto(500);
                        break;
                    case 15:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 8000), 90), telemetry);
                        break;
                    case 16:
                        state = RobotState.INTAKE;
                        waitAuto(1000);
                        break;
                    case 17:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0), 0), telemetry);
                        break;
                }
                break;
        }


        if (state != lastState) {
            firstTimeInIntake = true;
        }
        switch (state) {
            case INTAKE:
                if (firstTimeInIntake) {
                    startIntakeStartTime = timer.milliseconds();
                    firstTimeInIntake = false;
                }
                if (timer.milliseconds() - startIntakeStartTime > 500) {
                    elevatorState = ElevatorState.INTAKE;
                }
                if (Elevator.getElevatorPos() < ElevatorConstance.moveBoxMaxPos) {
                    wristState = WristState.MIDDLE;
                }
                if (Elevator.getElevatorPos() < ElevatorConstance.moveClawMaxPos) {
                    clawState = ClawState.OPEN;
                    if (timer.milliseconds() - startIntakeStartTime > 400) {
//                        intakeState = IntakeState.INTAKE;
                        wristState = WristState.INTAKE;
                    }
                } else {
                    clawState = ClawState.CLOSED;
                    intakeState = IntakeState.STOP;
                }
                break;
            case DROP:
                intakeState = IntakeState.STOP;
                clawState = ClawState.OPEN;
                break;
            case DROP_RIGHT:
                intakeState = IntakeState.STOP;
                clawState = ClawState.OPEN_RIGHT;
                break;
            case DROP_LEFT:
                intakeState = IntakeState.STOP;
                clawState = ClawState.OPEN_LEFT;
                break;
            case TRAVEL:
                firstTimeInTravel = lastState == RobotState.INTAKE;
                if (firstTimeInTravel) {
                    stopIntakeStartTime = timer.milliseconds();
                    firstTimeInTravel = false;
                    lastElevatorState = elevatorState;
                }
                clawState = ClawState.CLOSED;
                if (timer.milliseconds() - stopIntakeStartTime < 1000) {
                    elevatorState = ElevatorState.INTAKE;
                } else {
                    elevatorState = elevatorState == ElevatorState.INTAKE ? lastElevatorState : elevatorState;
                }
                if (timer.milliseconds() - stopIntakeStartTime > 600 && intakeState != IntakeState.DEPLETE) {
                    intakeState = IntakeState.STOP;
                }
                if (Elevator.getElevatorPos() > ElevatorConstance.moveBoxMinPos) {
                    wristState = WristState.DEPLETE;
                } else if (timer.milliseconds() - stopIntakeStartTime > 300) {
//                    wristState = WristState.GROUND;
                    wristState = WristState.MIDDLE;
                }
                break;
            case DEPLETE:
                if (elementPos == ElementPosition.MIDDLE) {
                    intakeState = IntakeState.DEPLETE_AUTO;
                } else {
                    intakeState = IntakeState.DEPLETE_AUTO;
                }
//                wristState = WristState.INTAKE;
                break;
            case CLIMB:
                intakeState = IntakeState.STOP;
                clawState = ClawState.CLOSED;
                elevatorState = ElevatorState.INTAKE;
                if (Elevator.getElevatorPos() < ElevatorConstance.moveBoxMaxPosClimb) {
                    wristState = WristState.INTAKE;
                }
                break;
        }


        Intake.operate(intakeState);
        Elevator.operate(elevatorState, gamepad1.right_stick_y, 0);
        Claw.operate(clawState);
        Wrist.operate(wristState);


        if (Drivetrain.isFinished) {
            actionNum++;
            Drivetrain.isFinished = false;
            Drivetrain.breakMotors();
            PoseTracker.resetPos();
        }
        lastState = state;
    }

    private static double startTime;
    private static boolean hasStarted = true;

    private static void waitAuto(int millis) {
        if (hasStarted) {
            startTime = timer.milliseconds();
            hasStarted = false;
        }
        if (millis < timer.milliseconds() - startTime) {
            actionNum++;
            Drivetrain.breakMotors();
            Drivetrain.resetEncoders();
            hasStarted = true;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        waitForStart();
        while (opModeIsActive()) {
            operate();
            telemetry.update();
        }
    }
}