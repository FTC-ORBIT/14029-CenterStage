package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
@TeleOp(name = "robot reset")
public class RobotReset extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Elevator.resetEncoder();
        waitForStart();
    }
}
