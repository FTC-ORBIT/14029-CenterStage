package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.autonomous.camera.BluePipeline;
import org.firstinspires.ftc.teamcode.autonomous.camera.Camera;
import org.firstinspires.ftc.teamcode.autonomous.camera.ElementPosition;
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
        while (opModeInInit()){
            telemetry.addData("element pos", BluePipeline.getElementPos());
            telemetry.update();
        }
    }

    public void operate() {
        actionNum = gamepad1.a ? 1 : actionNum;
        state = gamepad1.a ? RobotState.CLIMB : state;
        if (state == RobotState.CLIMB){
            state = RobotState.TRAVEL;
            elementPos = BluePipeline.getElementPos();
        }

        switch (elementPos){
            case RIGHT:
                switch (actionNum){
                    case 1:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0,38000), 0), telemetry);
                        telemetry.addData("posY", PoseTracker.getPose().getY());
                        telemetry.addData("posX", PoseTracker.getPose().getX());
                        telemetry.addData("angle", PoseTracker.getPose().getAngle());
                        break;
                    case 2:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0) , 90), telemetry);
                        break;
                    case 3:
                        state = RobotState.DEPLETE;
                        waitAuto(1000);
                        break;
                    case 4:
                        state = RobotState.TRAVEL;
                        Drivetrain.driveByTime(0.2);
                        waitAuto(400);
                        break;
                    case 5:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0,0), 0), telemetry);
                        state = RobotState.TRAVEL;
                        break;
                }
                break;
            case MIDDLE:
                switch (actionNum){
                    case 1:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0,30000), 0), telemetry);
                        state = RobotState.TRAVEL;
                        telemetry.addData("posY", PoseTracker.getPose().getY());
                        telemetry.addData("posX", PoseTracker.getPose().getX());
                        telemetry.addData("angle", PoseTracker.getPose().getAngle());
                        break;
                    case 2:
                        state = RobotState.DEPLETE;
                        waitAuto(1500);
                        break;
                    case 3:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0,6000), 0), telemetry);
                        state = RobotState.TRAVEL;
                        break;
                }
                break;
            case LEFT:
                switch (actionNum){
                    case 1:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 10000) , 0), telemetry);
                        break;
                    case 2:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0) , -30), telemetry);
                        break;
                    case 3:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 30000) , -30), telemetry);
                        break;
                    case 4:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, -7000) , -30), telemetry);
                        break;
                    case 5:
                        state = RobotState.DEPLETE;
                        waitAuto(1000);
                        break;
                    case 6:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, -8000) , -30), telemetry);
                        state = RobotState.TRAVEL;
                        break;
                    case 7:
                        Drivetrain.moveRobot(new Pose2D(new Vector(0, 0) , 0), telemetry);
                        state = RobotState.TRAVEL;
                        break;
                }
                break;
        }



        if (state != lastState){
            firstTimeInIntake = true;
        }
        switch (state){
            case INTAKE:
                if (firstTimeInIntake){
                    startIntakeStartTime = timer.milliseconds();
                    firstTimeInIntake = false;
                }
                if (timer.milliseconds() - startIntakeStartTime > 500){
                    elevatorState = ElevatorState.INTAKE;
                }
                if (Elevator.getElevatorPos() < ElevatorConstance.moveBoxMaxPos) {
                    wristState = WristState.MIDDLE;
                }
                if (Elevator.getElevatorPos() < ElevatorConstance.moveClawMaxPos){
                    clawState = ClawState.OPEN;
                    if (timer.milliseconds() - startIntakeStartTime > 400) {
                        intakeState = IntakeState.INTAKE;
                        wristState = WristState.INTAKE;
                    }
                }else {
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
                if (firstTimeInTravel){
                    stopIntakeStartTime = timer.milliseconds();
                    firstTimeInTravel = false;
                    lastElevatorState = elevatorState;
                }
                clawState = ClawState.CLOSED;
                if (timer.milliseconds() - stopIntakeStartTime < 1000){
                    elevatorState = ElevatorState.INTAKE;
                }else {
                    elevatorState = elevatorState == ElevatorState.INTAKE ? lastElevatorState : elevatorState;
                }
                if (timer.milliseconds() - stopIntakeStartTime > 600){
                    intakeState = IntakeState.STOP;
                }
                if (Elevator.getElevatorPos() > ElevatorConstance.moveBoxMinPos){
                    wristState = WristState.DEPLETE;
                }else if (timer.milliseconds() - stopIntakeStartTime > 300){
//                    wristState = WristState.GROUND;
                    wristState = WristState.MIDDLE;
                }
                break;
            case DEPLETE:
                if (elementPos == ElementPosition.MIDDLE){
                    intakeState = IntakeState.DEPLETE_AUTO;
                }else {
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


        if (Drivetrain.isFinished){
            actionNum++;
            Drivetrain.isFinished = false;
            Drivetrain.breakMotors();
            PoseTracker.resetPos();
        }
        lastState = state;
    }

    private static double startTime;
    private static boolean hasStarted = true;
    private static void waitAuto(int millis){
        if (hasStarted) {
            startTime = timer.milliseconds();
            hasStarted = false;
        }
        if (millis < timer.milliseconds() - startTime){
            actionNum++;
            Drivetrain.breakMotors();
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