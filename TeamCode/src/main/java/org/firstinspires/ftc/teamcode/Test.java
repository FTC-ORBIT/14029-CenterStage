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


    DcMotor motor;
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain.init(hardwareMap);
        Wrist.init(hardwareMap);
        Intake.init(hardwareMap);
        Gyro.init(hardwareMap);
        Gyro.resetGyro();
        PoseTracker.resetPos();
        Drivetrain.resetEncoders();
        Intake.resetEncoder();
        waitForStart();
        while (opModeIsActive()){
//            if (gamepad1.a) {
//                PoseTracker.update();
//            }
//            Drivetrain.operate(new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_trigger - gamepad1.left_trigger);
//            telemetry.addData("posY", PoseTracker.getPose().getY());
//            telemetry.addData("posX", PoseTracker.getPose().getX());
//            telemetry.addData("delta", Gyro.getDeltaAngle());
//            telemetry.addData("angle", Gyro.getAngle());
//            telemetry.update();
//            Gyro.setLastAngle(Gyro.getAngle());
            if (!gamepad1.a){
                Wrist.operate(WristState.INTAKE);
            }else {
                Wrist.operate(WristState.DEPLETE);
            }
        }

    }
}