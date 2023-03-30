package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PCMOne;
import frc.robot.Constants.CAN;

/** 
 * Subsystem for interacting with the robot's intake
 */
public class Intake extends SubsystemBase {
    
    private final CANSparkMax intakeMotor = new CANSparkMax(CAN.intakeMotor, MotorType.kBrushless);
    //                                                                          TODO: Is this the right module type?
    private final DoubleSolenoid intakePiston = new DoubleSolenoid(PCMOne.pcmOne, PneumaticsModuleType.CTREPCM, CAN.intakePistonForward, CAN.intakePistonReverse);

    private boolean intaking = false;
    
    /**
     * Creates a new Intake.
     */
    public Intake() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeMotor.setInverted(false);
    }

    // Self-explanatory functions
  
    public boolean getIntakingState() {
        return intaking;
    }

    public void setIntakingState(boolean state) {
        intaking = state;
    }

    public boolean getIntakePistonExtendStatus() {
        return intakePiston.get() == DoubleSolenoid.Value.kForward;
    }

    public void setintakePiston(boolean state) {
        intakePiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public void setIntakePercentOutput(double value) {
        intakeMotor.set(value);
    }

    private void updateSmartDashboard() {
        // SmartDashboardTab.putBoolean("Intake", "Intake State", getIntakingState());
        // SmartDashboardTab.putBoolean("Intake", "Pistons", getIntakePistonExtendStatus());
    }

    @Override
    public void periodic() {
        updateSmartDashboard();
    }
}
