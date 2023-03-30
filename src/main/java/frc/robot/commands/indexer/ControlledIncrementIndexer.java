/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Indexer;

/**
 * Waits 0.1 seconds after sensing a ball and then increments the indexer.
 */
public class ControlledIncrementIndexer extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Indexer m_indexer;
    
    private boolean tripped = false;
    private double startTime;

    /**
     * Waits 0.1 seconds after sensing a ball and then increments the indexer.
     *
     * @param indexer The indexer used by this command.
     */
    public ControlledIncrementIndexer(Indexer indexer) {
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
        if(m_indexer.getIntakeSensor() && !tripped) {
            tripped = true;
            startTime = Timer.getFPGATimestamp();
        }
        if(tripped)
            CommandScheduler.getInstance().schedule(new IncrementIndexer(m_indexer));

        if(Timer.getFPGATimestamp() - startTime > 0.1 && tripped) {
            tripped = false;
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
