package frc.vitruvianlib.utils;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.autonomous.VitruvianRamseteCommand;
import frc.robot.subsystems.DriveTrain;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class TrajectoryUtils {

    public static ArrayList<Pose2d> readCsvTrajectory(String filename) {
        BufferedReader reader;
        String fileLine;
        String[] fields;
        ArrayList<Pose2d> trajectoryPoints = new ArrayList<>();
        String fullpath = "/home/lvuser/deploy/Trajectories/" + filename + ".csv";
        try {
            reader = new BufferedReader(new FileReader(fullpath));
            while ((fileLine = reader.readLine()) != null) {
                fields = fileLine.split(",");
                trajectoryPoints.add(new Pose2d(Units.feetToMeters(Double.parseDouble(fields[0])),
                        Units.feetToMeters(Double.parseDouble(fields[1])),
                        Rotation2d.fromDegrees(Double.parseDouble(fields[2]))));

            }
        } catch (FileNotFoundException e) {
            System.out.println("Error: Could not find file");
            e.printStackTrace();
        } catch (IOException e) {
            System.out.println("Error: Could not read file");
            e.printStackTrace();
        }
        return trajectoryPoints;
    }

    public static VitruvianRamseteCommand generateRamseteCommand(DriveTrain driveTrain, ArrayList<Pose2d> path, TrajectoryConfig config) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(path, config);

        return generateRamseteCommand(driveTrain, trajectory);
    }

    public static VitruvianRamseteCommand generateRamseteCommand(DriveTrain driveTrain, Trajectory trajectory) {

        VitruvianRamseteCommand ramseteCommand = new VitruvianRamseteCommand(
                trajectory,
                driveTrain::getRobotPose2d,
                new RamseteController(),
                driveTrain.getFeedforward(),
                driveTrain.getDriveTrainKinematics(),
                driveTrain::getSpeedsMetersPerSecond,
                driveTrain.getLeftPIDController(),
                driveTrain.getRightPIDController(),
                driveTrain::setVoltageOutput,
                driveTrain
        );
        return ramseteCommand;
    }
}
