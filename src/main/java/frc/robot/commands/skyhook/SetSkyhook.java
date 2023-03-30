/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.skyhook;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

import java.util.function.DoubleSupplier;

/**
 * A command that sets the position of the skyhook only if the climber is extended
 */
public class SetSkyhook extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber m_climber;
  private final DoubleSupplier m_output;
  
  double value;

  /**
   * A command that sets the position of the skyhook only if the climber is extended.
   * 
   * @param climber The climber used by this command.
   * @param output The percent output to be sent to the skyhook.
   */
  public SetSkyhook(Climber climber, DoubleSupplier output) {
    m_climber = climber;
    m_output = output;
    // Use addRequirements() here to declare skyhook dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_climber.getClimbState())
      m_climber.setSkyhookPercentOutput(m_output.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setSkyhookPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}