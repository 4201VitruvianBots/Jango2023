/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * Tracks the target using vision and adjusts the turret to point towards it.
 */
public class AutoUseVisionCorrection extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Turret m_turret;
    private final Vision m_vision;
    private boolean turning;
    private double setpoint;

    /**
     * Tracks the target using vision and adjusts the turret to point towards it.
     *
     * @param turret The turret used by this command.
     * @param vision The vision used by this command.
     */
    public AutoUseVisionCorrection(Turret turret, Vision vision) {
        m_turret = turret;
        m_vision = vision;
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
        if(m_turret.getUsingSensor()) {
            if(m_vision.getValidTarget()) {
                if(! turning) {
                    m_vision.ledsOn();
                    setpoint = m_turret.getTurretAngleDegrees() + m_vision.getTargetX();

                    if(setpoint > m_turret.getMaxAngleDegrees()) {
                        setpoint -= 360;
                        if(setpoint < m_turret.getMinAngleDegrees())
                            setpoint = m_turret.getMinAngleDegrees();
                        turning = true;
                    } else if(setpoint < m_turret.getMinAngleDegrees()) {
                        setpoint += 360;
                        if(setpoint > m_turret.getMaxAngleDegrees())
                            setpoint = m_turret.getMaxAngleDegrees();
                        turning = true;
                    }
                } else {
                    if(m_turret.onTarget())
                        turning = false;
                }
            } else if(! m_vision.getValidTarget()) {
                setpoint = m_turret.getTurretAngleDegrees();
            }

            m_turret.setRobotCentricSetpointDegrees(setpoint);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(m_vision.getTargetX()) <= 3);
    }
}