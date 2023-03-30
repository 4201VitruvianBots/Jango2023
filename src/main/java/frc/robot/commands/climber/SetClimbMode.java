/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Sets climber mode
 */
public class SetClimbMode extends CommandBase {
    private final Climber m_climber;
    private final boolean m_mode;

    /**
     * Sets climber mode
     *
     * @param climber The subsystem used by this command.
     * @param mode true is up, false is down.
     */
    public SetClimbMode(Climber climber, boolean mode) {
        m_climber = climber;
        m_mode = mode;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climber);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_climber.setClimbState(m_mode);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
