/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Raises/lowers the climber based on joystick input
 */
public class SetClimberOutput extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber m_climber;
    private final DoubleSupplier m_joystickY; 

    private boolean currentDirection = true;
    private boolean movable, switchDirection;
    private double timestamp;
    private int lastDirection;

    /**
     * Creates a new SetClimberOutput.
     *
     * @param climber The climber used by this command.
     * @param xBoxController The joystick controller used by this command.
     */
    public SetClimberOutput(Climber climber, DoubleSupplier joystickY) {
        m_climber = climber;
        m_joystickY = joystickY;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double input = Math.abs(m_joystickY.getAsDouble()) > 0.2 ? m_joystickY.getAsDouble() : 0;
        
        int direction = input > 0 ? 1 : input < 0 ? - 1 : 0;
        // might be better
        if(m_climber.getClimbState()) {
            // SmartDashboardTab.putNumber("Climber", "Direction", direction);
            // SmartDashboardTab.putBoolean("Climber", "currentDirection", currentDirection);

            if(direction != lastDirection) {
                timestamp = Timer.getFPGATimestamp();
                movable = false;
                if(direction == 1 ) {
                    switchDirection = true;
                } else if(direction <= 0) {
                    switchDirection = false;
                }
            }

            if(movable) {
                double output = input;
                m_climber.setClimberPercentOutput(output);
            } else {
                if(switchDirection)
                    climberReleaseSequence();
                else {
                    m_climber.setClimbPiston(false);
                    movable = true;
                    currentDirection = true;
                }
            }
            lastDirection = direction;
        }
    }

    private void climberReleaseSequence() {
        // SmartDashboardTab.putString("Climber", "SetClimberOutput", "Releasing");
        m_climber.setClimbPiston(true);
        // m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.4);
        // m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.4);
        if(Math.abs(Timer.getFPGATimestamp() - timestamp) < 0.2)
            m_climber.setClimberPercentOutput(- 0.35);
        else if(Math.abs(Timer.getFPGATimestamp() - timestamp) < 0.4)
            m_climber.setClimberPercentOutput(0.25);
        else {
            m_climber.setClimberPercentOutput(0);
            movable = true;
            currentDirection = true;
            // m_controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            // m_controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
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
