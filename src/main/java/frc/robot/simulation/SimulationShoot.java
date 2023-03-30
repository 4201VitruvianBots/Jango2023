/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.simulation;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Moves a powercell across the screen as if it had been shot, for simulation
 */
public class SimulationShoot extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    FieldSim m_fieldSim;
    private static double lastShotTime;

    private boolean m_continuous;

    private Pose2d m_velocity;

    /**
     * Moves a powercell across the screen as if it had been shot, for simulation
     * 
     * @param fieldSim fieldSim to access the robot's powercells
     * @param continuous whether to keep running the command indefinitely (true), or just for an instant (false)
     */
    public SimulationShoot(FieldSim fieldSim, boolean continuous) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_fieldSim = fieldSim;
        m_continuous = continuous;
    }

    public void setVelocity(Pose2d velocity) {
        m_velocity = velocity;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentTime = RobotController.getFPGATime();
        // Shoot only every 80ms
        if(((currentTime - lastShotTime) / 1e6) > 0.080) {
            for(Powercell p: m_fieldSim.getPowerCells()) {
                if(p.getBallState() == 1 && !p.getBallShotState()) {
                    p.setBallShotState(true);
                    lastShotTime = currentTime;
                    p.setBallVelocity(m_velocity);
                    break;
                }
            }
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !m_continuous;
    }
}
