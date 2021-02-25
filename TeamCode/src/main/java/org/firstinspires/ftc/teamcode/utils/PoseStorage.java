package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class PoseStorage {
    public static Pose2d pose;
    String fileName = "PoseStorage.txt";
    Scanner input;
    FileWriter writer;
    public void storeInFile() throws IOException {
        writer = new FileWriter(fileName);
        writer.write(Double.toString(pose.getX()));
        writer.write(Double.toString(pose.getY()));
        writer.write(Double.toString(pose.getHeading()));
        writer.close();
    }
    public Pose2d parsePose() throws IOException {
        input = new Scanner(new File(fileName));
        String line = input.nextLine();
        String[] arr = line.split("\\s+");
        double x,y,heading;
        try{
            x = Double.parseDouble(arr[0]);
            y = Double.parseDouble(arr[1]);
            heading = Double.parseDouble(arr[2]);
        }
        catch(Exception e){
            x=0;
            y=0;
            heading=0;
        }
        return new Pose2d(x,y,heading);
    }
}
