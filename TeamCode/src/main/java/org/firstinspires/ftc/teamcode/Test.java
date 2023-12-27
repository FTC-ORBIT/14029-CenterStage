package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.utils.Vector;

@TeleOp(name = "Test")
public class Test extends OpMode {
    private final ElapsedTime timer = new ElapsedTime();
//    Elevator elevator = new Elevator();




    @Override
    public void init() {
//        Gyro.init(hardwareMap);
        Drivetrain.init(hardwareMap);
        Intake.init(hardwareMap);
//        elevator.init(hardwareMap);
    }

    @Override
    public void loop() {
        Drivetrain.operate(gamepad1);

        if (gamepad1.a){Intake.operate(IntakeState.INTAKE);}
        else {Intake.operate(IntakeState.STOP);}
        GlobalData.currentTime = timer.milliseconds();
        GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;
        GlobalData.lastTime = GlobalData.currentTime;

    }
}