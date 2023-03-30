/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

/**
 * Sends powercells to the shooter for 2 seconds.
 */
public class FeedAll extends CommandBase {
    private final Indexer m_indexer;

    /**
     * Sends powercells to the shooter for 2 seconds.
     *
     * @param indexer The indexer used by this command.
     */
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public FeedAll(Indexer indexer) {
        m_indexer = indexer;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(indexer);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_indexer.setIndexerPercentOutput(0.6);
        m_indexer.setKickerPercentOutput(0.5);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_indexer.setKickerPercentOutput(0);
        m_indexer.setIndexerPercentOutput(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double time = Timer.getFPGATimestamp();
        if(m_indexer.getIndexerTopSensor()) {
            time = Timer.getFPGATimestamp();
        }
        return time >= 2;
    }
}
