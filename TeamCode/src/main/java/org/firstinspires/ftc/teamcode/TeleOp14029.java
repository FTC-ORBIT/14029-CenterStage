package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.autonomous.aprilTagDetector.AprilTagDetector;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstance;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorState;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.plane.Plane;
import org.firstinspires.ftc.teamcode.robotSubSystems.poseTracker.PoseTracker;
import org.firstinspires.ftc.teamcode.robotSubSystems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.robotSubSystems.wrist.WristState;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.Vector;

@TeleOp(name = "TeleOp")
public class TeleOp14029 extends OpMode {
    private static final ElapsedTime timer = new ElapsedTime();

    RobotState state = RobotState.TRAVEL;

    ElevatorState elevatorState = ElevatorState.INTAKE;
    IntakeState intakeState = IntakeState.STOP;
    ClawState clawState = ClawState.CLOSED;
    WristState wristState = WristState.INTAKE;

    private static boolean planeState = false;


    @Override
    public void init() {
        Gyro.init(hardwareMap);
        Drivetrain.init(hardwareMap);
        Elevator.init(hardwareMap);
        Intake.init(hardwareMap);
        Wrist.init(hardwareMap);
        Claw.init(hardwareMap);
        Plane.init(hardwareMap);
    }
    private static double stopIntakeStartTime = 0;
    private static double startIntakeStartTime = 0;
    private static boolean firstTimeInIntake = true;

    private static boolean firstTimeInTravel = true;
    private static double startTimeInTravel = 0;
    private static RobotState lastState = RobotState.INTAKE;
    private static ElevatorState lastElevatorState = ElevatorState.INTAKE;
    @Override
    public void loop() {

        if (gamepad1.a){ state = RobotState.INTAKE; }
        if (gamepad1.b){ state = RobotState.DEPLETE; }
        if (gamepad1.x){ state = RobotState.DROP; }
        if (gamepad1.y){ state = RobotState.TRAVEL; }
        if (gamepad1.back) {Gyro.resetGyro();}
        if (gamepad2.back) {planeState = true;}
        if (gamepad2.a){planeState = false;}
        if (gamepad1.left_bumper){
            if (state == RobotState.DROP_RIGHT || state == RobotState.DROP) {
                state = RobotState.DROP;
            }else {
                state = RobotState.DROP_LEFT;
            }
        }
        if (gamepad1.right_bumper){
            if (state == RobotState.DROP_LEFT || state == RobotState.DROP) {
                state = RobotState.DROP;
            }else {
                state = RobotState.DROP_RIGHT;
            }
        }

        if (gamepad1.right_stick_button){ state = RobotState.CLIMB; }

        if (gamepad1.dpad_down){
            state = RobotState.TRAVEL;
            elevatorState = ElevatorState.LEVEL1;
        }
        if (gamepad1.dpad_right || gamepad1.dpad_left){
            state = RobotState.TRAVEL;
            elevatorState = ElevatorState.LEVEL2;
        }
        if (gamepad1.dpad_up){
            state = RobotState.TRAVEL;
            elevatorState = ElevatorState.LEVEL3;
        }

//        if (gamepad1.left_stick_button){wristState = WristState.GROUND;}



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
                }else if (timer.milliseconds() - stopIntakeStartTime > 400){
                    wristState = WristState.GROUND;
                }
                break;
            case DEPLETE:
                intakeState = IntakeState.DEPLETE;
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

        if (gamepad2.y){wristState = WristState.INTAKE;}
        if (gamepad2.b){Elevator.resetEncoder();}
        if (gamepad2.dpad_up){elevatorState = ElevatorState.MOVE;}
        lastState = state;

        telemetry.addData("DistSensor", Intake.distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("posElv", Elevator.getElevatorPos());
        telemetry.addData("power", Elevator.getElevatorPosL());
        telemetry.addData("pos", PoseTracker.getPose().vector);

        telemetry.addData("time", timer.milliseconds());

        Drivetrain.operate(new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_trigger - gamepad1.left_trigger);
        Intake.operate(intakeState);
        Elevator.operate(elevatorState, gamepad1.right_stick_y);
        Claw.operate(clawState);
        Intake.servoTest(gamepad2);
        Wrist.operate(wristState);
        Plane.operate(planeState);
        PoseTracker.update();
        Gyro.setLastAngle(Gyro.getAngle());
    }
}
