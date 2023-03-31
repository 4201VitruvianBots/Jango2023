/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

import java.util.function.DoubleSupplier;

/**
 * Sets the percent of voltage to be sent to the Climber.
 */
public class SetClimberManualOutput extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_Climber;
    private final DoubleSupplier m_percentOutput;
    private final double threshHold = 0.05;

    private double percentOutput;

    /**
     * Sets the percent of voltage to be sent to the Climber.
     *
     * @param Climber The Climber used by this command.
     * @param percentOutput The percent of voltage to send to the Climber.
     */
    public SetClimberManualOutput(Climber Climber, DoubleSupplier percentOutput) {
        m_Climber = Climber;
        m_percentOutput = percentOutput; 

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        percentOutput = m_percentOutput.getAsDouble(); 
        if(Math.abs(percentOutput) > threshHold){
            m_Climber.setClimberPercentOutput(percentOutput/1.25);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
