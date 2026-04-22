package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
@Config
public class SSConstants {

    //Tuning constants
    public static double tuningSpeed = 0.0;
    public static double tuningHoodPos = 0.5;

    //Shooter constants
    public static double kP = 0.0075;
    public static double kI = 0.00002;
    public static double kD = 0.0005;
    public static double kF = 0.0001458;
    public static double kS = 0.060;
    public static double kA = 0.1;
    public static double MAX_INTEGRAL = 5000;
    public static double nominalVoltage = 12.62;
    public static double minHoodPos = 0.5;
    public static double maxHoodPos = 1.0;
    public static double openGatePos = 0.5;
    public static double closeGatePos = 0.1;
    public static double rpmTol = 200;
    public static double k = 1173.61492;
    public static double radius = 10;
    public static double sotmMultiFactor = 1.0;

    //Intake constants
    public static double intakeSpeed = 1.0;
    public static double farTransferSpeed = 1.0;
    public static double closeTransferSpeed = 0.875;
    public static double outakeSpeed = -0.75;
    public static double currentSpike = 2500;
    public static double finalBallCurrentSpike = 4000;

    //Turret constants
    public static double maxAngle = 160;
    public static double minAngle = -maxAngle;
    public static double xOffset = -1.625;
    public static double yOffset = 0.0;

    //Field constants
    public static double goalY = 144;
    public static double redGoalX = 144;
    public static double blueGoalX = 0;

}
