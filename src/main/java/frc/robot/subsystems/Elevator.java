package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Simulation;

public class Elevator extends SubsystemBase {
    private double motorPercent = 0.0;
    private double positionMeters = 1.0;
    private double velocityMps = 0.0;

    private final double DT = 0.02;
    private final double MAX_ACCEL = 25.0;
    private final double GRAVITY = 9.81;

    public double getHeight() {
        return positionMeters;
    }

    public void setPercent(double percent) {
        motorPercent = MathUtil.clamp(percent, -1.0, 1.0);
    }

    @Override
    public void periodic() {
        // Run physics approximation
        double acceleration = (motorPercent * MAX_ACCEL) - GRAVITY;
        velocityMps = (velocityMps + acceleration * DT) * 0.99;
        positionMeters = MathUtil.clamp(positionMeters + (velocityMps * DT), 0.0, 1.0);

        if (positionMeters == 0.0 || positionMeters == 1.0)
            velocityMps = 0.0;

        // Sync with the visual Canvas
        Simulation.updateElevatorVisual(positionMeters);
    }
}