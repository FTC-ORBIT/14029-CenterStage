package org.firstinspires.ftc.teamcode.utils;

public class Pose2D {

    public Vector vector;
    double angle;

    public Pose2D(Vector vector, float angle){
        this.vector = vector;
        this.angle = angle;
    }

    public void setVector(final Vector vector){
        this.vector = vector;
    }

    public void setAngle(final double angle){
        this.angle = angle;
    }

    public void setX(final float x){
        vector.x = x;
    }

    public void setY(final float y){
        vector.y = y;
    }

    public double getX(){
        return vector.x;
    }

    public double getY(){
        return vector.y;
    }

    public void reset() {
        setX(0);
        setY(0);
        setAngle(0);
    }
}

