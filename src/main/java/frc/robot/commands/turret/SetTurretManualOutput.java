/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

/**
 * Sets the percent of voltage to be sent to the turret.
 */
public class SetTurretManualOutput extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret m_turret;
    private final double m_percentOutput;
    private final double threshHold = 0.05;

    /**
     * Sets the percent of voltage to be sent to the turret.
     *
     * @param turret The turret used by this command.
     * @param percentOutput The percent of voltage to send to the turret.
     */
    public SetTurretManualOutput(Turret turret, DoubleSupplier percentOutput) {
        m_turret = turret;
        m_percentOutput = percentOutput.getAsDouble();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(Math.abs(m_percentOutput) > threshHold)
            m_turret.setPercentOutput(m_percentOutput);
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
