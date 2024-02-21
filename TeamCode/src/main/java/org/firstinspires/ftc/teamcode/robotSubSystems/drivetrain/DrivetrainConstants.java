package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;

public class DrivetrainConstants {
    public static final double ticksPerRev = 537.7;
    public static final double wheelCircumferenceInCM = 9.6 * Math.PI;
    public static final double ticksToCM = wheelCircumferenceInCM / ticksPerRev;

    public static final double maxDriveSpeed = 1;
    public static final double minDriveSpeed = 0;

    public static final double moveRobotKp = 0.000025;
    public static final double moveRobotKi = 0;
    public static final double moveRobotKd = 0;
    public static final double moveRobotKf = 0;
    public static final double moveRobotIzone = 0;
    public static final double moveRobotMaxSpeed = 0.4;
    public static double moveRobotMinSpeed = 0.2;


    public static final double turnRobotKp = 0.02;
    public static final double turnRobotKi = 0;
    public static final double turnRobotKd = 0;
    public static final double turnRobotKf = 0;
    public static final double turnRobotIzone = 0;
    public static final double turnRobotMaxSpeed = 0.5;
    public static final double turnRobotMinSpeed = 0.3;



}
