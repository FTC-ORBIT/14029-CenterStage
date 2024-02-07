package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.aprilTagDetector.AprilTagDetector;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorConstance;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorState;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
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


    @Override
    public void init() {
        Gyro.init(hardwareMap);
        Drivetrain.init(hardwareMap);
        Elevator.init(hardwareMap);
        Intake.init(hardwareMap);
        Wrist.init(hardwareMap);
        Claw.init(hardwareMap);
    }
    private static double stopIntakeStartTime = 0;
    private static double startIntakeStartTime = 0;
    private static boolean firstTimeInIntake = true;
    private static RobotState lastState = RobotState.INTAKE.INTAKE;
    @Override
    public void loop() {

        if (gamepad1.a){ state = RobotState.INTAKE; }
        if (gamepad1.b){ state = RobotState.DEPLETE; }
        if (gamepad1.x){ state = RobotState.DROP; }
        if (gamepad1.y){ state = RobotState.TRAVEL; }
        if (gamepad1.back) {Gyro.resetGyro();}
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

        if (gamepad1.left_stick_button){wristState = WristState.GROUND;}



        if (state != lastState){
            firstTimeInIntake = true;
        }
        switch (state){
            case INTAKE:
                if (firstTimeInIntake){
                    startIntakeStartTime = timer.milliseconds();
                    firstTimeInIntake = false;
                }
                elevatorState = ElevatorState.INTAKE;
                if (Elevator.getElevatorPos() < ElevatorConstance.moveBoxMaxPos) {
                    wristState = WristState.INTAKE;
                }
                if (Elevator.getElevatorPos() < ElevatorConstance.moveClawMaxPos){
                    clawState = ClawState.OPEN;
                    if (timer.milliseconds() - startIntakeStartTime > 400) {
                        intakeState = IntakeState.INTAKE;
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
                if (firstTimeInIntake){
                    stopIntakeStartTime = timer.milliseconds();
                    firstTimeInIntake = false;
                }
                clawState = ClawState.CLOSED;
                if (timer.milliseconds() - stopIntakeStartTime > 400){
                    intakeState = IntakeState.STOP;
                }
                if (Elevator.getElevatorPos() > ElevatorConstance.moveBoxMinPos){
                    wristState = WristState.DEPLETE;
                }else if (timer.milliseconds() - stopIntakeStartTime > 100){
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
        lastState = state;

        telemetry.addData("pos", Elevator.getElevatorPos());
        telemetry.addData("power", Elevator.getElevatorPosL());

        telemetry.addData("time", timer.milliseconds());

        Drivetrain.operate(new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_trigger - gamepad1.left_trigger);
        Intake.operate(intakeState);
        Elevator.operate(elevatorState, gamepad1.right_stick_y);
        Claw.operate(clawState);
        Intake.servoTest(gamepad2);
        Wrist.operate(wristState);

    }
}
