package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.turret.SetTurretControlMode;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret;
/**
 * Disables control of the climber
 */
public class DisableClimbMode extends SequentialCommandGroup {
    /**
     * Disables control of the climber
     * 
     * @param climber The climber used by this command.
     * @param turret Lets the turret turn normally again (see {@link EnableClimbMode}).
     */
    public DisableClimbMode(Climber climber, Turret turret) {
        addCommands(new SetTurretControlMode(turret, 1),
                new SetClimbMode(climber, false),
                new RetractClimber(climber));
    }

}
