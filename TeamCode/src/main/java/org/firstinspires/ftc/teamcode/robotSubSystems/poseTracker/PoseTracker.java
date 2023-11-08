package org.firstinspires.ftc.teamcode.robotSubSystems.poseTracker;

import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Vector;

public class PoseTracker {
    private static Pose2D pose;
    public static Pose2D getPose(){
        return pose;
    }

    public static void update(){
        final Vector encoders = new Vector(Drivetrain.getXEncoderPos(), Drivetrain.getYEncoderPos());
        encoders.rotate(Angle.wrapAngle0_360(Gyro.getAngle()));

        pose.setVector(encoders);
        pose.setAngle(Angle.wrapAngle0_360(Gyro.getAngle()));
    }
}
