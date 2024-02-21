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





    private static Vector encoders = new Vector(0,0);

    private static Vector changed = encoders;




    public static void update(){
        encoders = Drivetrain.getEncoderPos();
        encoders.x -= PoseTrackerConstance.xErrorVal * Gyro.getDeltaAngle();
        encoders.y -= PoseTrackerConstance.yErrorVal * Gyro.getDeltaAngle();

        changed.x = encoders.x - lastXVAl;
        changed.y = encoders.y - lastYVAl;

        lastXVAl = encoders.x;
        lastYVAl = encoders.y;

//        changed = changed.rotate(Angle.wrapAngle0_360(Gyro.getAngle()));

        pose.setX((float) (pose.getX() + changed.x));
        pose.setY((float) (pose.getY() + changed.y));
        pose.setAngle(-Angle.wrapAngle0_360(Gyro.getAngle()));

        lastAngle = Gyro.getAngle();


    }

    public static void resetPos(){
        pose = new Pose2D(new Vector(0,0),0);
    }
}
