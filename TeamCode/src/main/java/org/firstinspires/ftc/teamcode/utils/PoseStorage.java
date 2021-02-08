package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.io.FileWriter;
import java.io.IOException;

public class PoseStorage {

    public static Pose2d currentPose = new Pose2d();


    public void setCurrentPose(Pose2d p){
        currentPose = p;

    }
}

