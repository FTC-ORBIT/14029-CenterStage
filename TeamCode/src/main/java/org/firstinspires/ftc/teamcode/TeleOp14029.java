package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.conveyor.ConveyorState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorState;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.sensors.Gyro;

@TeleOp(name = "TeleOp")
public class TeleOp14029 extends OpMode {
    private final ElapsedTime timer = new ElapsedTime();

    RobotState state = RobotState.TRAVEL;

    ElevatorState elevatorState = ElevatorState.INTAKE;
    IntakeState intakeState = IntakeState.STOP;


    @Override
    public void init() {
        Gyro.init(hardwareMap);
        Drivetrain.init(hardwareMap);
        Intake.init(hardwareMap);
    }

    @Override
    public void loop() {
        GlobalData.currentTime = timer.milliseconds();
        GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;
        GlobalData.lastTime = GlobalData.currentTime;

        Drivetrain.operate(gamepad1);

        switch (state){
            case INTAKE:
                elevatorState = ElevatorState.INTAKE;
                intakeState = IntakeState.INTAKE;
                break;
            case DROP:

                break;
            case TRAVEL:
                break;
        }


    }
}
