package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.utils.Vector;

@TeleOp(name = "TeleOp")
public class TeleOp14029 extends OpMode {
    private final ElapsedTime timer = new ElapsedTime();
    Drivetrain drivetrain = new Drivetrain();
    Elevator elevator = new Elevator();


    @Override
    public void init() {
        Gyro.init(hardwareMap);
        drivetrain.init(hardwareMap);
        elevator.init(hardwareMap);
    }

    @Override
    public void loop() {
        GlobalData.currentTime = timer.milliseconds();
        GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;
        GlobalData.lastTime = GlobalData.currentTime;

        drivetrain.operate(new Vector(-gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_trigger - gamepad1.left_trigger);
    }
}
