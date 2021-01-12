package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.io.FileWriter;
import java.io.IOException;

public class PoseStorageManager{
    public static String ps = "PoseStorage";
    FileWriter file;

    public PoseStorageManager() throws IOException {
        file = new FileWriter(ps);
    }
    public void setPose(Pose2d p) throws IOException {
        file = new FileWriter(ps);
        file.write(Double.toString(p.getX())+" ");
        file.append(Double.toString(p.getY())).append(" ");
        file.append(Double.toString(p.getHeading())).append(" ");
        file.close();

    }
}