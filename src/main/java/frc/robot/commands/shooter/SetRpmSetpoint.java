/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 * Sets and holds the shooter's RPM.
 */
public class SetRpmSetpoint extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;
  private double m_RPM;
  private boolean m_reset;
  /**
   * Sets and holds the shooter's RPM.
   * @param shooter the shooter subsystem
   * @param RPM the RPM to set
   * @param reset true = set the setpoint to 0 after the command finishes, false = keep the setpoint at our value after it finishes
   */
  public SetRpmSetpoint(Shooter shooter, double RPM, boolean reset) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_RPM = RPM;
    m_reset = reset;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setRpmSetpoint(m_RPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_reset) {
      m_shooter.setRpmSetpoint(0);
      m_shooter.setPercentOutput(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
