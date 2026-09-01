package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.NinjasLib.NinjasLogger;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.subsystems.Shooter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class Simulation {
    private static final List<GamePieceProjectile> balls = new ArrayList<>();

    // ---- Mechanism2D Visualization Layout ----
    private static Mechanism2d mechCanvas;

    // Elevator & Arm (Nested)
    private static MechanismRoot2d robotBaseRoot;
    private static MechanismLigament2d elevatorVisual;
    private static MechanismLigament2d armVisual;

    // Turret & Hood (Separated horizontally to prevent overlapping)
    private static MechanismRoot2d shooterBaseRoot;
    private static MechanismLigament2d hoodVisual;

    public static void setup() {
        balls.clear();

        // 1. Initialize Canvas (Width x Height)
        mechCanvas = new Mechanism2d(5.0, 5.0);

        // 2. Setup Elevator Root on the left side of the canvas
        robotBaseRoot = mechCanvas.getRoot("RobotBase", 2.5, 0);

        // Elevator extends straight up (90 degrees)
        elevatorVisual = robotBaseRoot.append(
            new MechanismLigament2d("ElevatorTower", 0.5, 90, 6, new Color8Bit(Color.kSteelBlue))
        );

        // Arm is nested to the elevator. It moves along with the elevator tower.
        armVisual = elevatorVisual.append(
            new MechanismLigament2d("ArmLinkage", 0.6, 0, 6, new Color8Bit(Color.kOrangeRed))
        );

        // 3. Setup Turret/Shooter Root on the right side of the canvas to avoid overlapping visual lines
        shooterBaseRoot = mechCanvas.getRoot("ShooterBase", 2.8, 0.25);

        // Hood mounted onto the turret
        hoodVisual = shooterBaseRoot.append(
            new MechanismLigament2d("HoodAngle", 0.4, 20, 4, new Color8Bit(Color.kYellow))
        );
    }

    public static void updateElevatorVisual(double currentHeightMeters) {
        elevatorVisual.setLength(Math.max(0.01, currentHeightMeters));
    }

    public static void updateArmVisual(double angleRad) {
        armVisual.setAngle(Math.toDegrees(angleRad) - 90);
    }

    public static void updateHoodVisual(double angleRad) {
        hoodVisual.setAngle(Math.toDegrees(angleRad));
    }

    public static void launchBall() {
        // Convert the primitive double turret angle to a proper Rotation2d object
        Rotation2d turretOffset = RobotContainer.getTurret().getAngle();
        Rotation2d totalLaunchHeading = RobotState.get().getRobotPose().getRotation().plus(turretOffset);

        // Calculate flywheel surface speed matching your mechanical configurations
        double conversionFactor = edu.wpi.first.math.util.Units.inchesToMeters(Shooter.WHEEL_RADIUS_INCH) * 2 * Math.PI * Shooter.WHEEL_FRICTION;
        double speedMps = RobotContainer.getShooter().getVelocity() * conversionFactor;

        GamePieceProjectile ball = new RebuiltFuelOnFly(
            RobotState.get().getRobotPose().getTranslation(),
            new Translation2d(0.1, 0.0),
            Swerve.getInstance().getSpeeds().getAsFieldRelative(),
            totalLaunchHeading,
            Units.Meters.of(0.481),
            Units.MetersPerSecond.of(speedMps),
            Units.Degrees.of(RobotContainer.getHood().getAngle().getDegrees())
        );

        balls.add(ball);
        SimulatedArena.getInstance().addGamePieceProjectile(ball);
    }

    public static void periodic() {
        SimulatedArena.getInstance().simulationPeriodic();

        // Clear out dead projectiles that fall below the field floor
        Iterator<GamePieceProjectile> iterator = balls.iterator();
        while (iterator.hasNext()) {
            GamePieceProjectile ball = iterator.next();
            if (ball.getPose3d().getZ() <= 0.07) {
                SimulatedArena.getInstance().removePiece(ball);
                iterator.remove();
            }
        }

        // Log outputs to AdvantageScope
        Pose3d[] ballPoses = balls.stream().map(GamePieceProjectile::getPose3d).toArray(Pose3d[]::new);
        NinjasLogger.log("Simulation/Balls", ballPoses);
        SmartDashboard.putData("Simulation/Mechanism2d", mechCanvas);
    }
}