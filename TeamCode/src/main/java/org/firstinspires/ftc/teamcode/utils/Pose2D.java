package org.firstinspires.ftc.teamcode.utils;

public class Pose2D {
    public double x;
    public double y;
    public double rot;
    public Pose2D(double x, double y, double rot){
        this.x = x;
        this.y = y;
        this.rot = rot;
    }
    public Pose2D(Vector2D v, double rot){
        this.x = v.x;
        this.y = v.y;
        this.rot = rot;
    }
}
