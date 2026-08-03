package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIO;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIOPS5;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.*;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private LoggedCommandController driverController;
    private LoggedCommandController operatorController;

    private static Elevator elevator;
    private static Arm arm;
    private static Shooter shooter;
    private static Hood hood;
    private static Turret turret;
    private static Indexer indexer;
    private static SwerveSubsystem swerveSubsystem;

    public RobotContainer() {
        if (!GeneralConstants.kRobotMode.isReplay()) {
            driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIOPS5(GeneralConstants.kDriverControllerPort));
//            operatorController = new LoggedCommandController("Operator", new LoggedCommandControllerIOPS5(GeneralConstants.kOperatorControllerPort));
        } else {
            driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIO() {});
//            operatorController = new LoggedCommandController("Operator", new LoggedCommandControllerIO() {});
        }

        swerveSubsystem = new SwerveSubsystem(true, driverController::getLeftX, driverController::getLeftY, driverController::getRightX, driverController::getRightY);
        RobotStateBase.set(new RobotState(SubsystemConstants.kSwerve.chassis.kinematics));

        elevator = new Elevator();
        arm = new Arm();
        shooter = new Shooter();
        hood = new Hood();
        turret = new Turret();
        indexer = new Indexer();

        Simulation.setup();
        configureBindings();
        
    }

    public static Elevator getElevator() {
        return elevator;
    }
    public static Arm getArm() {
        return arm;
    }
    public static Shooter getShooter() {
        return shooter;
    }
    public static Hood getHood() {
        return hood;
    }
    public static Turret getTurret() {
        return turret;
    }
    public static Indexer getIndexer() {
        return indexer;
    }
    public static SwerveSubsystem getSwerveSubsystem() {
        return swerveSubsystem;
    }
    private final KrakenSubsystem m_krakenSubsystem = new KrakenSubsystem();
    private void configureBindings() {
        driverController.triangle().whileTrue(m_krakenSubsystem.runSpeedCommand(0.5));
        // Runs highGoalCommand if isFarAway() is true, otherwise runs lowGoalCommand


        driverController.cross().onTrue(Commands.either(
                Commands.sequence(
                        Commands.startRun( () ->
                        elevator.setHeight(0.8),
                                () -> {
                                    if (elevator.getHeight() > 0.4) {
                                        arm.setAngle(Rotation2d.fromDegrees(30.0));
                                    }
                                },
                                elevator, arm
                        );






    }


// דוגמה: כשלוחצים על כפתור cross בשלט, המנוע ירוץ ב-50% כוח. כשעוזבים - הוא יעצור אוטומטית!

    public void controllerPeriodic() {
        driverController.periodic();
//        operatorController.periodic();
    }

    public void periodic() {
        Simulation.periodic();
        elevator.logError();
        SmartDashboard.putNumber("Arm/angle", arm.getAngle().getDegrees());
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
