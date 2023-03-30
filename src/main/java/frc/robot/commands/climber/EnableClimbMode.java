package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.turret.SetTurretControlMode;
import frc.robot.commands.turret.SetTurretRobotRelativeAngle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret;

/**
 * Enables control of the climber
 */
public class EnableClimbMode extends SequentialCommandGroup {
    /**
     * Enables control of the climber
     * 
     * @param climber The climber used by this command.
     * @param turret Turns the turret sideways so it won't be hit by the climber.
     */
    public EnableClimbMode(Climber climber, Turret turret) {
        addCommands(new SetTurretRobotRelativeAngle(turret, 95),
                new SetTurretControlMode(turret, 0),
                new SetClimbMode(climber, true),
                new ExtendClimber(climber));
    }

}
