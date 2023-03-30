/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

import java.util.function.DoubleSupplier;

/**
 * Raises/lowers the climber based on joystick input
 */
public class SetClimberOutputOld extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_climber;
    private final DoubleSupplier m_input;

    private boolean currentDirection = true;
    private boolean movable, switchDirection;
    private double timestamp;
    private int direction;

    /**
     * Raises/lowers the climber based on joystick input
     *
     * @param climber The climber used by this command.
     * @param input Supplier for controlling the motor
     */
    public SetClimberOutputOld(Climber climber, DoubleSupplier input) {
        m_climber = climber;
        m_input = input;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double input = Math.abs(m_input.getAsDouble()) > 0.2 ? m_input.getAsDouble() : 0;
        direction = input > 0 ? 1 : input < 0 ? - 1 : 0;
        if(m_climber.getClimbState()) {
            if(direction != 0) {
                timestamp = Timer.getFPGATimestamp();
                if(direction == 1 && ! currentDirection) {
                    movable = false;
                    switchDirection = true;
                } else if(direction <= 0 && currentDirection) {
                    movable = false;
                    switchDirection = false;
                }
            }

            if(movable) {
                double output = (m_climber.getClimberPosition() < - 512) && (input < 0) ? 0 : input;
                m_climber.setClimberPercentOutput(output);
            } else {
                if(switchDirection)
                    climberReleaseSequence();
                else
                    climberRetractSequence();
            }
        }
    }

    private void climberReleaseSequence() {
        m_climber.setClimbPiston(true);

        if(Math.abs(Timer.getFPGATimestamp() - timestamp) < 0.2)
            m_climber.setClimberPercentOutput(- 0.25);
        else if(Math.abs(Timer.getFPGATimestamp() - timestamp) < 0.4)
            m_climber.setClimberPercentOutput(0.25);
        else {
            m_climber.setClimberPercentOutput(0);
            movable = true;
            currentDirection = true;
        }
    }

    private void climberRetractSequence() {
        m_climber.setClimbPiston(false);
        if(Math.abs(Timer.getFPGATimestamp() - timestamp) < 0.2)
            m_climber.setClimberPercentOutput(- 0.25);
        else {
            m_climber.setClimberPercentOutput(0);
            movable = true;
            currentDirection = false;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_climber.setClimberPercentOutput(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
