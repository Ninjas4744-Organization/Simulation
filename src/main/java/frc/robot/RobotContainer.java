package frc.robot;

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
    double errorSum = 0;
    double lastTimeStamp = 0;
    double kP = 1;
    double kI = 0.2551020408163265;
    double kD = 0;
    double kIZone = 0.5;
    private final KrakenSubsystem m_krakenSubsystem = new KrakenSubsystem();
    private void configureBindings() {
        driverController.cross().whileTrue(m_krakenSubsystem.runSpeedCommand(0.5));
        driverController.triangle().toggleOnTrue(Commands.startRun(
                () -> {
                    errorSum = 0;
                    lastTimeStamp = Timer.getFPGATimestamp();

                },
                () -> {
            double error = 0.8 - elevator.getHeight();
            double dT = Timer.getFPGATimestamp() - lastTimeStamp;
            if (error < kIZone) {
                errorSum += error * dT;
            }
            double output = error * kP;
            double outputSpeed = output + kI * errorSum;
            elevator.setPercent(outputSpeed);
            lastTimeStamp = Timer.getFPGATimestamp();
                    SmartDashboard.putNumber("Elevator/Error", error);

        }));
    }
// דוגמה: כשלוחצים על כפתור A בשלט, המנוע ירוץ ב-6 וולט. כשעוזבים - הוא יעצור אוטומטית!

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
