package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
    public static final class USB {
        public static final int leftJoystick = 0;
        public static final int rightJoystick = 1;
        public static final int xBoxController = 2;    
    }

    public static final class CAN {
        public static final int leftFrontDriveMotor = 20;
        public static final int leftRearDriveMotor = 21;
        public static final int rightFrontDriveMotor = 22;
        public static final int rightRearDriveMotor = 23;

        public static final int ledPort = 0;

        public static final int indexerMotor = 35;
        public static final int kickerMotor = 36;
        public static final int intakeSensor = 0;
        public static final int indexerTopSensor = 1;
        public static final int indexerBottomSensor = 2;

        public static final int flywheelMotorA = 40;
        public static final int flywheelMotorB = 41;

        public static final int intakeMotor = 30;
        public static final int intakePistonForward = 2;
        public static final int intakePistonReverse = 3;

        public static final int climbMotorA = 50;
        public static final int climbMotorB = 51;
        public static final int skyhookMotor = 55;
        public static final int climbPistonForward = 4;
        public static final int climbPistonReverse = 5;

        public static final int turretMotor = 60;
        public static final int turretEncoder = 61;

        public static final int turretHomeSensor = 3;
    }

    public static final class PCMOne {
        public static final int pcmOne = 11;
    }

    public static final class DIO {
        public static final int turretHome = 3;
    }


    public static final class Drive {
        public static final int[] kLeftEncoderPorts = new int[]{10, 11};
        public static final int[] kRightEncoderPorts = new int[]{12, 13};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final double kTrackWidthMeters = Units.inchesToMeters(21.5);
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackWidthMeters);

        public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);
        public static final double kDriveGearing = 8.0;

        public static final int kMagEncoderCPR = 4096;
        public static final int kFalconEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = Units.feetToMeters(0.5);
        public static final double kEncoderDistancePerPulseMeters =
                // Encoders are not on the wheel shaft for Falcons, so need to multiply by gear ratio
                (kWheelDiameterMeters * Math.PI) / (double) kFalconEncoderCPR * kDriveGearing;
        public static final double kEncoderDistancePerPulseMetersSim =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kMagEncoderCPR * kDriveGearing;

        public static final boolean kGyroReversed = true;

        public static final double ksVolts = 0.75514;
        public static final double kvVoltSecondsPerMeter = 1;
        public static final double kaVoltSecondsSquaredPerMeter = 0.57574;

        // These two values are "angular" kV and kA
        public static final double kvVoltSecondsPerRadian = 3.34;
        public static final double kaVoltSecondsSquaredPerRadian = 0.19;

        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
                LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
                        kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);

        public static final double kMaxVelocityMetersPerSecond = 2.0;
    }

    public static final class Indexer {
        public static final double kI_Zone = 1;
        public static final double maxVel = 1.1e4;
        public static final double maxAccel = 1e6;
        public static final double gearRatio = 1.0 / 27.0;

        // PID terms/other constants
        public static double kF = 0.0001;
        public static double kP = 0.000001;
        public static double kI = 80;
        public static double kD = 0.0001;
    }

    public static final class Intake {
        public enum IntakeStates {
            INTAKE_EMPTY, INTAKE_ONE_BALL, INTAKE_FOUR_BALLS, INTAKE_FIVE_BALLS
        }
    }
    
    public static class Climber {

    }
    
    public static class Turret {    
        public static final int encoderUnitsPerRotation = 4096;
    
        // Turret PID gains
        public static final double kF = 0.07;
        public static final double kP = 0.2;
        public static final double kI = 0.0015;
        public static final double kD = 0.0;
        public static final int kI_Zone = 900;
        public static final int kMaxIAccum = 1000000;
        public static final int kErrorBand = 50;
        public static final int kCruiseVelocity = 10000; // TODO: What units?
        public static final int kMotionAcceleration = kCruiseVelocity * 10; // TODO: What units?
        public static final double minAngleDegrees = - 90;
        public static final double maxAngleDegrees = 90;
        public static final double gearRatio = 18.0 / 120.0;
    }

    public static final class Shooter {
        public static final double kFlywheelDiameterMeters = Units.inchesToMeters(4); //10.16 cm
        public static final double kFlywheelGearRatio = 1.0; //10.16 cm
        public static final int kFalconEncoderCPR = 2048;

        public static final double kS = 0.155;
        public static final double kV = 0.111;
        public static final double kA = 0.02;

        public static final double kDriveSimEncoderDistancePerPulse = (kFlywheelDiameterMeters * Math.PI) / ((double) kFalconEncoderCPR * kFlywheelGearRatio);

    }

    public static class SimConstants {
        public static final Pose2d blueGoalPoseMeters = new Pose2d(0, 5.831, new Rotation2d());
    }
}