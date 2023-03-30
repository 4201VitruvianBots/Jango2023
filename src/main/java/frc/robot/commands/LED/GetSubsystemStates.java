/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

/**
 * Sets the LED based on the subsystems' statuses
 */
public class GetSubsystemStates extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final LED m_led;
    private final Indexer m_indexer;
    private final Intake m_intake;
    private final Vision m_vision;
    private final Turret m_turret;
    private final Climber m_climber;
    // private final Controls m_controls;

    /**
     * Sets the LED based on the subsystems' statuses
     */
    public GetSubsystemStates(LED led, Indexer indexer, Intake intake, Vision vision, Turret turret, Climber climber) {
        m_led = led;
        m_indexer = indexer;
        m_intake = intake;
        m_vision = vision;
        m_turret = turret;
        m_climber = climber;
        // m_controls = controls;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(led);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_led.setRGB(0, 125, 0);
        m_led.setSolidColor();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_led.setRGB(75, 20, 150);
        m_led.setSolidColor();

        if(! RobotContainer.getInitializationState()) { // robot is initializing
            m_led.setState(- 1);
        } else {
            if(DriverStation.isDisabled()) {
                if(isRobotReady())
                    m_led.setState(8);
                else
                    m_led.setState(7);
            } else {
                if(m_climber.getClimbState()) {
                    m_led.setState(0);
                } else if(m_intake.getIntakingState()) {
                    if(m_indexer.getIndexerTopSensor() && m_indexer.getIndexerBottomSensor() && m_indexer.getIntakeSensor()) {
                        m_led.setState(2);
                    } else if(m_indexer.newBall()) {
                        m_led.setState(3);
                    } else {
                        m_led.setState(4);
                    }
                } else {
                    if(m_vision.hasTarget()) {
                        m_led.setState(5);
                    } else if(! m_vision.hasTarget()) {
                        m_led.setState(6);
                    }
                }
            }
        }
    }

    private boolean isRobotReady() {
        // && PSI is high (?). Save for comp
        return (DriverStation.isFMSAttached() && m_turret.getInitialHome()) || m_turret.getInitialHome();// && PSI is high (?). Save for comp
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

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
