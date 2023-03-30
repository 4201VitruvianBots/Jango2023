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
 * Sets the percent output and pistons of the intake.
 */
public class SetIntakeStates extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_intake;
    private final boolean m_extend;
    private final double m_percentOutput;

    /**
     * Sets the percent output and pistons of the intake.
     *
     * @param intake The subsystem used by this command.
     * @param extend Whether or not to extend the intake.
     * @param percentOutput The speed for the intake to spin at.
     */
    public SetIntakeStates(Intake intake, boolean extend, double percentOutput) {
        m_intake = intake;
        m_extend = extend;
        m_percentOutput = percentOutput;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(m_intake.getIntakePistonExtendStatus() != m_extend)
            m_intake.setintakePiston(m_extend);
        m_intake.setIntakePercentOutput(m_percentOutput);
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
