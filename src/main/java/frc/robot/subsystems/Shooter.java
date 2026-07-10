package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private double motorPercent = 0.0;
    private double currentVelocityRPS = 0.0;

    private final double DT = 0.02;
    private final double MAX_VELOCITY_RPS = 100.0;
    public static final double WHEEL_RADIUS_INCH = 2.0;
    public static final double WHEEL_FRICTION = 0.5;

    public void setPercent(double percent) {
        this.motorPercent = Math.max(-1.0, Math.min(1.0, percent));
    }

    public double getVelocity() {
        return currentVelocityRPS;

    }

    @Override
    public void periodic() {
        // Spin up behaves like a standard linear system approaching terminal velocity
        double targetVelocity = motorPercent * MAX_VELOCITY_RPS;
        currentVelocityRPS = currentVelocityRPS + (targetVelocity - currentVelocityRPS) * 0.15;

        Logger.recordOutput("Shooter/VelocityRPS", currentVelocityRPS);

        double conversionFactor = edu.wpi.first.math.util.Units.inchesToMeters(Shooter.WHEEL_RADIUS_INCH) * 2 * Math.PI * Shooter.WHEEL_FRICTION;
        double speedMps = RobotContainer.getShooter().getVelocity() * conversionFactor;
        Logger.recordOutput("Shooter/VelocityMps", speedMps);
    }
}