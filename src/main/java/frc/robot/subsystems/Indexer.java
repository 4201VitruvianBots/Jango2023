package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

/**
 * Susbsystem for interacting with the robot's indexer (feeds balls from intake to shooter)
 */
public class Indexer extends SubsystemBase {

    // Setup indexer motor controller (SparkMax)
    CANSparkMax master = new CANSparkMax(CAN.indexerMotor, MotorType.kBrushless);
    RelativeEncoder encoder = master.getEncoder();
    SparkMaxPIDController pidController = master.getPIDController();
    VictorSPX kicker = new VictorSPX(CAN.kickerMotor);
    // Indexer sensors setup
    DigitalInput intakeSensor = new DigitalInput(CAN.intakeSensor);
    DigitalInput indexerTopSensor = new DigitalInput(CAN.indexerTopSensor);
    DigitalInput indexerBottomSensor = new DigitalInput(CAN.indexerBottomSensor);
    // Detect whether a new ball has been picked up
    // There is a new ball if the intake sensor is blocked and was not blocked before
    boolean pTripped = false;

    /**
     * Creates a new Indexer.
     */
    public Indexer() {
        // Motor and PID controller setup
        master.restoreFactoryDefaults();
        master.setInverted(false);

        master.setIdleMode(IdleMode.kBrake);

        pidController.setFF(Constants.Indexer.kF);
        pidController.setP(Constants.Indexer.kP);
        pidController.setI(Constants.Indexer.kI);
        pidController.setD(Constants.Indexer.kD);
        pidController.setSmartMotionMaxVelocity(Constants.Indexer.maxVel, 0); // Formerly 1.1e4
        pidController.setSmartMotionMaxAccel(Constants.Indexer.maxAccel, 0); // Formerly 1e6
        pidController.setSmartMotionAllowedClosedLoopError(1, 0);
        pidController.setIZone(Constants.Indexer.kI_Zone);

        kicker.configFactoryDefault();
        kicker.setInverted(true);
    }

    public boolean getIntakeSensor() {
        return (! intakeSensor.get());
    }

    public boolean getIndexerBottomSensor() {
        return ! indexerBottomSensor.get();
    }

    public boolean getIndexerTopSensor() {
        return ! indexerTopSensor.get();
    }

    public void setKickerPercentOutput(double output) {
        kicker.set(ControlMode.PercentOutput, output);
    }

    /**
     * @param output Speed value from -1 to 1
     */
    public void setIndexerPercentOutput(double output) {
        master.set(output);
    }

    public boolean newBall() {
        boolean returnVal;
        returnVal = pTripped == false && getIntakeSensor();
        pTripped = getIntakeSensor();
        return returnVal;
    }

    public void setRPM(double rpm) {
        double setpoint = rpm / Constants.Indexer.gearRatio;
        SmartDashboard.putNumber("Indexer Setpoint", setpoint);
        pidController.setReference(setpoint, ControlType.kSmartVelocity);
    }

    private void initShuffleboard() {
        // Unstable. Don''t use until WPILib fixes this
        Shuffleboard.getTab("Indexer").addBoolean("Intake Sensor", this :: getIntakeSensor);
        Shuffleboard.getTab("Indexer").addBoolean("Indexer Bottom Sensor", this :: getIndexerBottomSensor);
        Shuffleboard.getTab("Indexer").addBoolean("Indexer Top Sensor", this :: getIndexerTopSensor);
    }

    private void updateSmartDashboard() {
        // SmartDashboardTab.putBoolean("Indexer", "Intake Sensor", getIntakeSensor());
        // SmartDashboardTab.putBoolean("Indexer", "Indexer Bottom Sensor", getIndexerBottomSensor());
        // SmartDashboardTab.putBoolean("Indexer", "Indexer Top Sensor", getIndexerTopSensor());
    }

    private void updatePIDValues() {
        // Allow PID values to be set through SmartDashboard
        Constants.Indexer.kF = SmartDashboard.getNumber("kF", 0);
        Constants.Indexer.kP = SmartDashboard.getNumber("kP", 0);
        Constants.Indexer.kI = SmartDashboard.getNumber("kI", 0);
        Constants.Indexer.kD = SmartDashboard.getNumber("kD", 0);
        pidController.setFF(Constants.Indexer.kF);
        pidController.setP(Constants.Indexer.kP);
        pidController.setI(Constants.Indexer.kI);
        pidController.setD(Constants.Indexer.kD);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateSmartDashboard();
    }
}
