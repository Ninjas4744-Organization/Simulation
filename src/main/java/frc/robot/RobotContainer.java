package frc.robot;

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

    private void configureBindings() {
        driverController.triangle().whileTrue(Commands.startEnd(() -> elevator.setPercent(0.5), () -> elevator.setPercent(0)));

        driverController.circle().whileTrue(Commands.startEnd(() -> arm.setPercent(0.5), () -> arm.setPercent(0)));
        driverController.square().whileTrue(Commands.startEnd(() -> arm.setPercent(-0.5), () -> arm.setPercent(0)));

        driverController.R1().whileTrue(Commands.startEnd(() -> turret.setPercent(-0.5), () -> turret.setPercent(0)));
        driverController.L1().whileTrue(Commands.startEnd(() -> turret.setPercent(0.5), () -> turret.setPercent(0)));

        driverController.L2().whileTrue(Commands.startEnd(() -> hood.setPercent(0.5), () -> hood.setPercent(0)));

        driverController.R2().whileTrue(Commands.startEnd(() -> {
            shooter.setPercent(0.5);
            indexer.setPercent(1);
        }, () -> {
            shooter.setPercent(0);
            indexer.setPercent(0);
        }));
    }

    public void controllerPeriodic() {
        driverController.periodic();
//        operatorController.periodic();
    }

    public void periodic() {
        Simulation.periodic();
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
