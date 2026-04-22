package org.firstinspires.ftc.teamcode.math;

import static org.firstinspires.ftc.teamcode.subsystem.SSConstants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystem.SSConstants;

public class SpeedController {

    private final VoltageSensor voltageSensor;

    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastVel = 0.0;
    private double lastTarget = 0.0;

    private long lastTimeNs = -1;

    // Tunables (add to constants if you want)
    private static double kV = kF;   // velocity FF
    private static double kA = SSConstants.kA; // acceleration FF (tune this)
    private static final double MAX_ACCEL = 6000; // RPM/s (tune)
    private static final double VELOCITY_ALPHA = 0.7; // filtering

    public SpeedController(HardwareMap hardwareMap) {
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public double calculate(double targetVel, double currentVelRaw) {

        long nowNs = System.nanoTime();

        if (lastTimeNs < 0) {
            lastTimeNs = nowNs;
            lastVel = currentVelRaw;
            return 0.0;
        }

        double dt = (nowNs - lastTimeNs) * 1e-9;
        lastTimeNs = nowNs;

        if (dt <= 0) return 0.0;

        // --- Velocity filtering ---
        double currentVel = VELOCITY_ALPHA * lastVel + (1 - VELOCITY_ALPHA) * currentVelRaw;

        // --- Target ramping ---
        double maxDelta = MAX_ACCEL * dt;
        double deltaTarget = targetVel - lastTarget;
        deltaTarget = Math.max(-maxDelta, Math.min(maxDelta, deltaTarget));
        double profiledTarget = lastTarget + deltaTarget;

        // --- Acceleration ---
        double accel = (profiledTarget - lastTarget) / dt;

        // --- Error ---
        double error = profiledTarget - currentVel;

        // --- Derivative (on measurement) ---
        double derivative = -(currentVel - lastVel) / dt;

        // --- Conditional integral ---
        double outputEstimate = kP * error;
        boolean allowIntegral = Math.abs(outputEstimate) < 1.0;

        if (allowIntegral) {
            integral += error * dt;
            integral = Math.max(-MAX_INTEGRAL, Math.min(integral, MAX_INTEGRAL));
        }

        // --- Feedforward ---
        double ff = kF * profiledTarget + SSConstants.kA * accel;
        double ks = (Math.abs(profiledTarget) > 50) ? Math.copySign(kS, ff) : 0.0;

        // --- PID ---
        double output =
                kP * error +
                        kI * integral +
                        kD * derivative +
                        ff +
                        ks;

        // --- Voltage compensation (global) ---
        double voltage = voltageSensor.getVoltage();
        if (voltage > 0.1) {
            output *= nominalVoltage / voltage;
        }

        // --- Clamp ---
        output = Math.max(-1.0, Math.min(output, 1.0));

        // --- Save state ---
        lastError = error;
        lastVel = currentVel;
        lastTarget = profiledTarget;

        return output;
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        lastVel = 0.0;
        lastTarget = 0.0;
        lastTimeNs = -1;
    }
}