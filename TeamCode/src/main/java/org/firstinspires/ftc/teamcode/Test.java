package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.aprilTagDetector.AprilTagDetector;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.ClawState;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.ElevatorState;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.poseTracker.PoseTracker;
import org.firstinspires.ftc.teamcode.robotSubSystems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.robotSubSystems.wrist.WristState;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.utils.Vector;

@TeleOp(name = "Test")
public class Test extends LinearOpMode {

    private static final ElapsedTime timer = new ElapsedTime();
    private static double startTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain.init(hardwareMap);
        Gyro.init(hardwareMap);
        waitForStart();
        startTime = timer.milliseconds();
        while (opModeIsActive() && timer.milliseconds() - startTime < 3500){
            Drivetrain.operate(new Vector(0.4, -0.1), 0);
        }
//        while (opModeIsActive() && timer.milliseconds() - startTime < 1000){
//            Drivetrain.operate(new Vector(0, 0), 0.2);
//        }

    }
}