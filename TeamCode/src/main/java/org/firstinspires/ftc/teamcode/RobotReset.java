package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
@Autonomous(name = "robot reset")
public class RobotReset extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Elevator.init(hardwareMap);
        waitForStart();
        Elevator.resetEncoder();
    }
}
