
package org.firstinspires.ftc.teamcode.math;

import static org.firstinspires.ftc.teamcode.subsystem.SSConstants.k;
import static org.firstinspires.ftc.teamcode.subsystem.SSConstants.sotmMultiFactor;

public class ShooterEquation {
    public double getTargetRPM(double distance) {
        return ((0.0236063 * distance * distance)
                + (4.21139 * distance)
                + k) * (60.0 / 28.0);
    }

    public double getHoodPos(double distance) {
        return 1;
    }

    public double getAirTime(double distance) {
        return  sotmMultiFactor * ((6.66667e-8) * Math.pow(distance, 3) - 0.00003 * Math.pow(distance, 2) + 0.00633333 * distance - 0.1);
    }
}
