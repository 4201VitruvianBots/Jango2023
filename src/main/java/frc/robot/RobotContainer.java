// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.USB;
import frc.robot.commands.autonomous.routines.DriveForwardDistance;
import frc.robot.commands.climber.EnableClimbMode;
import frc.robot.commands.climber.SetClimberManualOutput;
import frc.robot.commands.climber.SetClimberOutput;
import frc.robot.commands.drivetrain.SetArcadeDrive;
import frc.robot.commands.indexer.EjectAll;
import frc.robot.commands.indexer.FeedAll;
import frc.robot.commands.intake.ControlledIntake;
import frc.robot.commands.intake.SetIntakePercentOutput;
import frc.robot.commands.intake.ToggleIntakePistons;
import frc.robot.commands.shooter.RapidFireSetpoint;
import frc.robot.commands.shooter.SetRpmSetpoint;
import frc.robot.commands.turret.SetTurretManualOutput;
import frc.robot.commands.turret.SetTurretSetpointFieldAbsolute;
import frc.robot.commands.turret.ToggleTurretUsingSensor;
import frc.robot.commands.turret.ZeroTurretEncoder;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.vitruvianlib.utils.JoystickWrapper;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveTrain m_driveTrain = new DriveTrain();
    private final Intake m_intake = new Intake();
    private final Indexer m_indexer = new Indexer();
    private final Turret m_turret = new Turret(m_driveTrain);
    private final Vision m_vision = new Vision(m_driveTrain, m_turret);
    private final Shooter m_shooter = new Shooter(m_vision);
    private final Climber m_climber = new Climber();

    private FieldSim m_fieldSim;

    static Joystick leftJoystick = new Joystick(Constants.USB.leftJoystick);
    static Joystick rightJoystick = new Joystick(Constants.USB.rightJoystick);
    public CommandXboxController xboxController = new CommandXboxController(USB.xBoxController);
  

    public Trigger[] leftTriggers = new Trigger[2];
    public Trigger[] rightTriggers = new Trigger[2];
    public Trigger[] xBoxTriggers = new Trigger[10];
    public Trigger[] xBoxPOVTriggers = new Trigger[4];
    public Trigger[] leftJoystickTriggers = new Trigger[2]; // left joystick buttons
    public Trigger[] rightJoystickTriggers = new Trigger[2]; // right joystick buttons
  
    private static boolean init = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        initializeSubsystems();

        // Configure the button bindings
        configureBindings();
    }

    public static boolean getInitializationState() {
        return init;
    }

    public static void setInitializationState(boolean state) {
        init = state;
    }

    public void initializeSubsystems() {
        m_fieldSim = new FieldSim(m_driveTrain, m_turret);
        m_driveTrain.setDefaultCommand(
            new SetArcadeDrive(m_driveTrain,
                () -> -leftJoystick.getRawAxis(1),
                () -> rightJoystick.getRawAxis(0)));

        // m_turret.setDefaultCommand(new SetTurretManualOutput(m_turret, () -> xboxController.getLeftY()));
        m_turret.setDefaultCommand(new SetTurretManualOutput(m_turret, xboxController::getLeftX));

        m_climber.setDefaultCommand(new SetClimberManualOutput(m_climber, xboxController::getLeftY));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureBindings() { 
        for (int i = 0; i < leftJoystickTriggers.length; i++)
          leftJoystickTriggers[i] = new JoystickButton(leftJoystick, (i + 1));
        for (int i = 0; i < rightJoystickTriggers.length; i++)
          rightJoystickTriggers[i] = new JoystickButton(rightJoystick, (i + 1));
    
    
    
        // xboxController.leftTrigger().whileTrue(new SetIntakePercentOutput(m_intake, 0.5));
        // xboxController.rightTrigger().whileTrue(new SetIntakePercentOutput(m_intake, 0.5));
        xboxController.rightTrigger().whileTrue(new FeedAll(m_indexer)); 
        xboxController.leftTrigger().whileTrue(new EjectAll(m_indexer, m_intake)); 

        xboxController.x().whileTrue(new RapidFireSetpoint(m_shooter, m_indexer, m_intake, 0.1));
        xboxController.a().whileTrue(new RapidFireSetpoint(m_shooter, m_indexer, m_intake, 0.32)); 
        xboxController.b().whileTrue(new RapidFireSetpoint(m_shooter, m_indexer, m_intake, 0.54)); 
        //xboxController.y().whileTrue(new RapidFireSetpoint(m_shooter, m_indexer, m_intake, 0.45)); 
        
        xboxController.rightBumper().onTrue(new ToggleIntakePistons(m_intake));
        // xboxController.rightTrigger().whileTrue(new ControlledIntake(m_intake, m_indexer, xboxController::getRightY)); // Deploy intake

        // xBoxButtons[0].whileHeld(new SetRpmSetpoint(m_shooter, 3600, true)); // [A] Short-range
        // xBoxButtons[1].whileHeld(new SetRpmSetpoint(m_shooter, 3800, true)); // [B] Med-range
        // xBoxButtons[3].whileHeld(new SetRpmSetpoint(m_shooter, 4000, true)); // [Y] Long-range
        xboxController.povDown().onTrue(new EjectAll(m_indexer, m_intake));      //Top POV - Eject All

        // xboxController.rightTrigger().whileTrue(new RapidFireSetpoint(m_shooter, m_indexer, m_intake));        // flywheel on toggle

        xboxController.start().onTrue(new ToggleTurretUsingSensor(m_turret));                        // start - toggle control mode turret
        xboxController.leftStick().onTrue(new EnableClimbMode(m_climber, m_turret));                     // R3 - toggle driver climb mode?

        xboxController.povRight().onTrue(new ZeroTurretEncoder(m_turret));
      }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new DriveForwardDistance(m_driveTrain, m_fieldSim, 2);
    }

    public void teleOpInit() {
        m_driveTrain.resetEncoderCounts();
        m_driveTrain.setDriveTrainNeutralMode(0); // Half and half
    }


    public void autonomousInit() {
        if (RobotBase.isReal()) {
            m_driveTrain.resetEncoderCounts();
            m_driveTrain.resetOdometry(m_driveTrain.getRobotPose2d(), m_fieldSim.getRobotPoseMeters().getRotation());
        } else {
            m_fieldSim.initSim();
            m_driveTrain.resetEncoderCounts();
            m_driveTrain.resetOdometry(m_fieldSim.getRobotPoseMeters(), m_fieldSim.getRobotPoseMeters().getRotation());
        }
    }

    public DriveTrain getRobotDrive() {
        return m_driveTrain;
    }

    public void simulationInit() {
        m_fieldSim.initSim();
        //m_driveTrain.setSimPose(new Pose2d(5,5, new Rotation2d()));
    }

    public void simulationPeriodic() {
        if (!RobotState.isTest())
            m_fieldSim.simulationPeriodic();
    }
}
