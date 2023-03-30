package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

/**
 * Sets the desired angle of the turret relative to the robot's angle.
 */
public class SetTurretRobotRelativeAngle extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret m_turret;
    private final double m_setpoint;

    /**
     * Sets the desired angle of the turret relative to the robot's angle.
     *
     * @param turret The turret used by this command.
     * @param setpoint The setpoint to set the turret to turn towards the front of the robot.
     */
    public SetTurretRobotRelativeAngle(Turret turret, double setpoint) {
        m_turret = turret;
        m_setpoint = setpoint;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_turret.setRobotCentricSetpointDegrees(m_setpoint);
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
        return Math.abs(m_turret.getTurretAngleDegrees() - m_setpoint) < 1;
    }
}
