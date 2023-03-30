package frc.robot.commands.autonomous.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.SetOdometry;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.vitruvianlib.utils.TrajectoryUtils;

import java.util.ArrayList;
import java.util.Arrays;

public class AllyTrenchPathStraightSim extends SequentialCommandGroup {
    public AllyTrenchPathStraightSim(DriveTrain driveTrain, FieldSim fieldSim) {
        Pose2d startPosition = new Pose2d(12.5, 0.736995, new Rotation2d());
        Pose2d[] startToTrenchPathPoints = {
                startPosition,
                new Pose2d(8.5, 0.736995, new Rotation2d(0))
        };
        Pose2d[] trenchToShootPathPoints = {
                new Pose2d(8.5, 0.736995, new Rotation2d(0)),
                startPosition
        };
        TrajectoryConfig configA = new TrajectoryConfig(Units.feetToMeters(6), Units.feetToMeters(4));
        configA.setReversed(true);
        configA.setEndVelocity(0);
        configA.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configA.getMaxVelocity()));
        configA.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(), 10));

        var startToTrenchPath = TrajectoryGenerator.generateTrajectory(Arrays.asList(startToTrenchPathPoints), configA);

        var startToTrenchCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, startToTrenchPath);

        var configB = new TrajectoryConfig(Units.feetToMeters(8), Units.feetToMeters(4));
        configB.setReversed(false);
        configB.setEndVelocity(0);
        configB.addConstraint(new DifferentialDriveKinematicsConstraint(driveTrain.getDriveTrainKinematics(), configB.getMaxVelocity()));
        configB.addConstraint(new DifferentialDriveVoltageConstraint(driveTrain.getFeedforward(), driveTrain.getDriveTrainKinematics(), 10));

        var trenchToShootPath = TrajectoryGenerator.generateTrajectory(Arrays.asList(trenchToShootPathPoints), configB);
        var trenchToShootCommand = TrajectoryUtils.generateRamseteCommand(driveTrain, trenchToShootPath);

        addCommands(
                new SetOdometry(driveTrain, fieldSim, startPosition),
                startToTrenchCommand,
                trenchToShootCommand.andThen(() -> driveTrain.setMotorTankDrive(0, 0))
        );
    }
}
