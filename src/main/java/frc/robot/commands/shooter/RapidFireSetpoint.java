/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Feeds to shooter when it's ready to shoot
 */
public class RapidFireSetpoint extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Shooter m_shooter;
    private double m_output; 

    private double startTime;

    /**
     * Feeds to shooter when it's ready to shoot
     *
     */
    public RapidFireSetpoint(Shooter shooter, double shooterOutput) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_shooter = shooter;

        m_output = shooterOutput; 
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
            m_shooter.setPercentOutput(m_output);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        m_shooter.setPercentOutput(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (false);
    }
}
