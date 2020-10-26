package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DashboardVars {
    public static double shooterMaxVel=1;
    public static double F = 32767.0/shooterMaxVel;
    public static double P = 0.1 * F;
    public static double I = 0.1*P;
    public static double D = 0;
    public static PIDFCoefficients shooter = new PIDFCoefficients(P,I,D,F);
}
