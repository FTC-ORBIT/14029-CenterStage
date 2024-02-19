package org.firstinspires.ftc.teamcode.robotSubSystems.poseTracker;

import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Vector;

public class PoseTracker {
    private static Pose2D pose = new Pose2D(new Vector(0,0), 0);
    public static Pose2D getPose(){
        return pose;
    }

    private static double xErrorVal = 0;
    private static double yErrorVal = 0;

    private static double lastAngle = 0;
    private static double lastXVAl = 0;
    private static double lastYVAl = 0;

    private static double changedXVAl = 0;
    private static double changedYVAl = 0;


    private static Vector encoders = new Vector(0,0);



    public static void update(){
        encoders = Drivetrain.getEncoderPos();
        encoders.x += PoseTrackerConstance.xErrorVal * Gyro.getDeltaAngle();
        encoders.y += PoseTrackerConstance.yErrorVal * Gyro.getDeltaAngle();

        changedXVAl = encoders.x - lastXVAl;
        changedYVAl = encoders.y - lastYVAl;

        lastXVAl = encoders.x;
        lastYVAl = encoders.y;

        encoders.rotate(Angle.wrapAngle0_360(Gyro.getAngle()));

        pose.setX((float) (pose.getX() + changedXVAl));
        pose.setY((float) (pose.getY() + changedYVAl));
        pose.setAngle(Angle.wrapAngle0_360(Gyro.getAngle()));

        lastAngle = Gyro.getAngle();

    }
}
