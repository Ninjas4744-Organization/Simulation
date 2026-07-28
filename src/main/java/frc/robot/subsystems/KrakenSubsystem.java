package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KrakenSubsystem extends SubsystemBase {

    // 1. האובייקט של המנוע הפיזי (TalonFX)
    private final TalonFX m_motor = new TalonFX(1); // 1 הוא ה-CAN ID לדוגמה

    // 1. הפעלה לפי אחוז כוח (בין 1.0- ל-1.0+)
    public void setSpeed(double percent) {
        m_motor.setControl(m_dutyCycle.withOutput(percent));
    }

    // 2. הפעלה לפי מתח מדויק בוולטים (בין 12.0- ל-12.0+)
    public void setVoltage(double volts) {
        m_motor.setControl(m_voltage.withOutput(volts));
    }

    // 3. עצירת המנוע
    public void stop() {
        m_motor.stopMotor();
    }
    // 1. פקודה להרצת המנוע במתח מסוים כל עוד לוחצים על כפתור
    public Command runVoltageCommand(double volts) {
        return run(() -> setVoltage(volts))
                .finallyDo(() -> stop());
    }

    // 2. פקודה להרצת המנוע באחוז כוח מסוים
    public Command runSpeedCommand(double percent) {
        return run(() -> setSpeed(percent))
                .finallyDo(() -> stop());
    }

    // 2. אובייקטי השליטה (נכונים מראש בזיכרון)
    private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0);
    private final VoltageOut m_voltage = new VoltageOut(0);
    // 2. בנאי (Constructor) - מגדיר את המנוע ברגע פתיחת הסאב-סיסטם
    public KrakenSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // א. כיוון סיבוב ומצב עצירה
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Brake = בלימה חזקה | Coast = החלקה חופשית

        // ב. הגנת זרם (Stator Current Limit)
        config.CurrentLimits.StatorCurrentLimit = 40.0; // מגבלה של 40 אמפר להגנה על הגיר והמנוע
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // ג. שליחת ההגדרות למנוע הפיזי
        m_motor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        // קריאת נתונים בזמן אמת מהמנוע (Phoenix 6)
        double currentPosition = m_motor.getPosition().getValueAsDouble();
        double currentVelocity = m_motor.getVelocity().getValueAsDouble();
        double statorCurrent = m_motor.getStatorCurrent().getValueAsDouble();

        // שליחת הנתונים ל-SmartDashboard
        SmartDashboard.putNumber("Kraken/Position", currentPosition);
        SmartDashboard.putNumber("Kraken/Velocity", currentVelocity);
        SmartDashboard.putNumber("Kraken/Stator Current", statorCurrent);
    }

}
