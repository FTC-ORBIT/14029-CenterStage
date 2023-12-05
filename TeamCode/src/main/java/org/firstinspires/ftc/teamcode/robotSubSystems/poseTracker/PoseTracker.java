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

    private static double xErrorVal = 0;
    private static double yErrorVal = 0;

    private static double lastAngle = 0;


    public static void update(){
        final Vector encoders = new Vector(
                Drivetrain.getXEncoderPos() - xErrorVal * (Gyro.getAngle() - lastAngle),
                Drivetrain.getYEncoderPos() - yErrorVal * (Gyro.getAngle() - lastAngle));

        encoders.rotate(Angle.wrapAngle0_360(Gyro.getAngle()));

        pose.setVector(encoders);
        pose.setAngle(Angle.wrapAngle0_360(Gyro.getAngle()));

        lastAngle = Gyro.getAngle();
    }
}
