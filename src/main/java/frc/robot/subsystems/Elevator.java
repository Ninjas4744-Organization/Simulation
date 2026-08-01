package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Simulation;

public class Elevator extends SubsystemBase {
    private double motorPercent = 0.0;

    // Simulation parameters
    private static final double kGearRatio = 6.0;
    private static final double kCarriageMassKg = 7.0;
    private static final double kDrumRadiusMeters = 0.025;
    private static final double kMinHeightMeters = 0.0;
    private static final double kMaxHeightMeters = 1.0;

    private final ElevatorSim m_elevatorSim = new ElevatorSim(
        DCMotor.getKrakenX60(1),
        kGearRatio,
        kCarriageMassKg,
        kDrumRadiusMeters,
        kMinHeightMeters,
        kMaxHeightMeters,
        true,
        0
    );

    public double getHeight() {
        return m_elevatorSim.getPositionMeters();
    }
    PIDController pid = new PIDController(7.5, 0.01, 0.5);

    public Command setHeight(double height) {

        return Commands.run(() -> {
            System.out.println("print hello world");
            setPercent(pid.calculate(getHeight(), height));
        });
    }
    public void logError() {
        SmartDashboard.putNumber("Elevator/error", pid.getError() );
        SmartDashboard.putNumber("Elevator/ErrorRate", pid.getErrorDerivative());
    }
    public void setPercent(double percent) {
        this.motorPercent = MathUtil.clamp(percent, -1.0, 1.0);
    }

    @Override
    public void periodic() {
        // Apply motor output (converted to voltage, max 12V)
        m_elevatorSim.setInputVoltage(motorPercent * 12.0);

        // Advance the simulation by the standard 20ms periodic step
        m_elevatorSim.update(0.02);

        // Sync with the visual Canvas
        Simulation.updateElevatorVisual(getHeight());
    }
}