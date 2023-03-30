/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/**
 * Sets the percent of the total voltage sent to the intake.
 */
public class SetIntakePercentOutput extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_intake;
    private final double m_percentOutput;

    /**
     * Sets the percent of the total voltage sent to the intake.
     *
     * @param intake The intake used by this command.
     * @param percentOutput The percent of voltage to be sent to the intake.
     */
    public SetIntakePercentOutput(Intake intake, double percentOutput) {
        m_intake = intake;
        m_percentOutput = percentOutput;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.setIntakingState(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.setIntakePercentOutput(m_percentOutput);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_intake.setIntakePercentOutput(0);
        m_intake.setIntakingState(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
