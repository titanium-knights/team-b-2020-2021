package org.firstinspires.ftc.teamcode.utils;

public class LaunchMath {
    private double distanceToGoal;
    final double g = 9.80665;
    public LaunchMath(double distanceToGoal){
        this.distanceToGoal = distanceToGoal;
    }
    public void setDistanceToGoalMeters(double distanceToGoal){
        this.distanceToGoal = distanceToGoal;
    }
    public void setDistanceToGoalInches(double distanceToGoal){
        this.distanceToGoal = distanceToGoal/39.37;
    }
    //Tangential Speed at which flywheel needs to spin at in m/s
    public double getLinearVelocity(){
        double vy = Math.sqrt(0.9017*2*this.g);
        double time = vy/this.g;
        double vx = this.distanceToGoal/time;
        return Math.sqrt(Math.pow(vx,2)+Math.pow(vy,2));
    }
    //Angle of Launcher
    public double getTheta(){
        double vy = Math.sqrt(0.9017*2*this.g);
        double time = vy/this.g;
        double vx = this.distanceToGoal/time;
        return Math.atan2(vy,vx);
    }
}
