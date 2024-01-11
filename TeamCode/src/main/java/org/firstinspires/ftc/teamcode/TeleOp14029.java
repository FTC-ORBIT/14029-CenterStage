package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotData.GlobalData;
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

@TeleOp(name = "TeleOp")
public class TeleOp14029 extends OpMode {
    private final ElapsedTime timer = new ElapsedTime();

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

    @Override
    public void loop() {
//        GlobalData.currentTime = timer.milliseconds();
//        GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;
//        GlobalData.lastTime = GlobalData.currentTime;

        if (gamepad1.a){
            state = RobotState.INTAKE;
        }
        if (gamepad1.b){
            state = RobotState.DEPLETE;
        }
        if (gamepad1.x){
            state = RobotState.DROP;
        }
        if (gamepad1.y){
            state = RobotState.TRAVEL;
        }
        if (gamepad1.left_bumper){
            state = RobotState.CLIMB;
        }

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

        switch (state){
            case INTAKE:
                elevatorState = ElevatorState.INTAKE;
                intakeState = IntakeState.INTAKE;
                if (Elevator.getElevatorPos() < ElevatorConstance.moveBoxMaxPos) {
                    wristState = WristState.INTAKE;
                }
                if (Elevator.getElevatorPos() < ElevatorConstance.moveClawMaxPos){
                    clawState = ClawState.OPEN;
                }else {
                    clawState = ClawState.CLOSED;
                }
                break;
            case DROP:
                intakeState = IntakeState.STOP;
                clawState = ClawState.OPEN;
                wristState = WristState.DEPLETE;
                break;
            case TRAVEL:
                clawState = ClawState.CLOSED;
                intakeState = IntakeState.STOP;
                if (Elevator.getElevatorPos() > ElevatorConstance.moveBoxMinPos){
                    wristState = WristState.DEPLETE;
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
                if (Elevator.getElevatorPos() < ElevatorConstance.moveBoxMaxPos) {
                    wristState = WristState.INTAKE;
                }
                break;
        }

        Drivetrain.operate(gamepad1);
        Intake.operate(intakeState);
        Elevator.operate(elevatorState,gamepad1);
        Claw.operate(clawState);
        Wrist.operate(wristState);
        telemetry.addData("", Elevator.getElevatorPos());

    }
}
