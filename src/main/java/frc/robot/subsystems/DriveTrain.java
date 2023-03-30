// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Drive;

/**
 * A differential drivetrain with two falcon motors on each side
 */
public class DriveTrain extends SubsystemBase {

    private final double gearRatio = 1.0 / 8.0;

    private double kS = Drive.ksVolts;
    private double kV = Drive.kvVoltSecondsPerMeter;
    private double kA = Drive.kaVoltSecondsSquaredPerMeter;

    public double kP = 2.0; //3.6294;
    public double kI = 0;
    public double kD = 0;
    public int controlMode = 0;

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
    DifferentialDriveOdometry odometry;
    DifferentialDrivePoseEstimator m_poseEstimator;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    PIDController leftPIDController = new PIDController(kP, kI, kD);
    PIDController rightPIDController = new PIDController(kP, kI, kD);

    private final TalonFX[] driveMotors = {
            new TalonFX(Constants.CAN.leftFrontDriveMotor),
            new TalonFX(Constants.CAN.leftRearDriveMotor),
            new TalonFX(Constants.CAN.rightFrontDriveMotor),
            new TalonFX(Constants.CAN.rightRearDriveMotor)
    };
    double m_leftOutput, m_rightOutput;

    private final boolean[] brakeMode = {
            true,
            false,
            true,
            false
    };

    private final AHRS navX = new AHRS(SerialPort.Port.kMXP);

    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();


    // Temporary until CTRE supports FalconFX in WPILib Sim
    private final TalonSRX[] simMotors = new TalonSRX[4];

    public DifferentialDrivetrainSim m_drivetrainSimulator;
    private ADXRS450_GyroSim m_gyroAngleSim;

    public DriveTrain() {
        // Set up DriveTrain motors
        configureCtreMotors(driveMotors);

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeadingDegrees()), getEncoderCount(0), getEncoderCount(2));

        m_poseEstimator = new DifferentialDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(getHeadingDegrees()), getEncoderCount(0), getEncoderCount(2), getRobotPose2d());
        System.out.println(odometry); 


        if (RobotBase.isSimulation()) {
            for (int i = 0; i < 4; i++)
                simMotors[i] = new TalonSRX(24 + i);
            configureCtreMotors(simMotors);
            simMotors[0].setSensorPhase(true);
            simMotors[2].setSensorPhase(false);

            m_drivetrainSimulator = new DifferentialDrivetrainSim(
                    Drive.kDrivetrainPlant,
                    Drive.kDriveGearbox,
                    Drive.kDriveGearing,
                    Drive.kTrackWidthMeters,
                    Drive.kWheelDiameterMeters / 2.0,
                    null);//VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));


            m_gyroAngleSim = new ADXRS450_GyroSim(m_gyro);
        }
        SmartDashboard.putData("DT Subsystem", this);
    }

    public void configureCtreMotors(BaseTalon... motors) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].configFactoryDefault();

            motors[i].configOpenloopRamp(0.1);
            motors[i].configClosedloopRamp(0.1);
            motors[i].setNeutralMode(NeutralMode.Coast);
            motors[i].configForwardSoftLimitEnable(false);
            motors[i].configReverseSoftLimitEnable(false);

            if (motors[i] instanceof TalonFX) {
                driveMotors[i].configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
                driveMotors[i].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            } else if (motors[i] instanceof TalonSRX) {
                simMotors[i].configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
                simMotors[i].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
            }
        }

        motors[0].setInverted(true);
        motors[1].setInverted(true);
        motors[2].setInverted(false);
        motors[3].setInverted(false);

        motors[0].setSensorPhase(false);
        motors[2].setSensorPhase(false);

        motors[1].set(ControlMode.Follower, driveMotors[0].getDeviceID());
        motors[3].set(ControlMode.Follower, driveMotors[2].getDeviceID());
        motors[1].setNeutralMode(NeutralMode.Brake);
        motors[3].setNeutralMode(NeutralMode.Brake);

        motors[1].configOpenloopRamp(0);
        motors[3].configOpenloopRamp(0);
    }

    public double getEncoderCount(int sensorIndex) {
        return driveMotors[sensorIndex].getSelectedSensorPosition();
    }

    public double getHeadingDegrees() {
        if (RobotBase.isReal())
            return Math.IEEEremainder(-navX.getAngle(), 360);
        else
            return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Drive.kGyroReversed ? -1.0 : 1.0);
    }

    public void resetAngle() {
        navX.zeroYaw();
    }

    public void setNavXOffsetDegrees(double angle) {
        navX.setAngleAdjustment(angle);
    }

    public double getWheelDistanceMeters(int sensorIndex) {

        if (RobotBase.isReal())
            return driveMotors[sensorIndex].getSelectedSensorPosition() * Drive.kEncoderDistancePerPulseMeters;
        else {
            return simMotors[sensorIndex].getSelectedSensorPosition() * Drive.kEncoderDistancePerPulseMetersSim;
        }
    }

    public double getMotorInputCurrentAmps(int motorIndex) {
        return driveMotors[motorIndex].getSupplyCurrent();
    }

    public void resetEncoderCounts() {
        driveMotors[0].setSelectedSensorPosition(0);
        driveMotors[2].setSelectedSensorPosition(0);
        if (RobotBase.isSimulation()) {
            simMotors[0].getSimCollection().setQuadratureRawPosition(0);
            simMotors[2].getSimCollection().setQuadratureRawPosition(0);
        }
    }

    public void setMotorArcadeDrive(double throttle, double turn) {
        double leftPWM = throttle + turn;
        double rightPWM = throttle - turn;

        // Normalization
        double magnitude = Math.max(Math.abs(leftPWM), Math.abs(rightPWM));
        if (magnitude > 1.0) {
            leftPWM *= 1.0 / magnitude;
            rightPWM *= 1.0 / magnitude;
        }

        setMotorVelocityMetersPerSecond(leftPWM * Drive.kMaxVelocityMetersPerSecond, rightPWM * Drive.kMaxVelocityMetersPerSecond);
    }

    public void setMotorTankDrive(double leftOutput, double rightOutput) {
        setMotorVelocityMetersPerSecond(leftOutput * Drive.kMaxVelocityMetersPerSecond, rightOutput * Drive.kMaxVelocityMetersPerSecond);
    }

    public void setVoltageOutput(double leftVoltage, double rightVoltage) {
        var batteryVoltage = RobotController.getBatteryVoltage();
        if (Math.max(Math.abs(leftVoltage), Math.abs(rightVoltage))
                > batteryVoltage) {
            leftVoltage *= batteryVoltage / 12.0;
            rightVoltage *= batteryVoltage / 12.0;
        }
        // smartDashboard.putNumber("DriveTrain", "Left Voltage", leftVoltage);
        // smartDashboard.putNumber("DriveTrain", "Right Voltage", rightVoltage);

        setMotorPercentOutput(leftVoltage / batteryVoltage, rightVoltage / batteryVoltage);

    }

    private void setMotorPercentOutput(double leftOutput, double rightOutput) {
        m_leftOutput = leftOutput;
        m_rightOutput = rightOutput;
        driveMotors[0].set(ControlMode.PercentOutput, leftOutput);
        driveMotors[2].set(ControlMode.PercentOutput, rightOutput);

        if (RobotBase.isSimulation()) {
            simMotors[0].set(ControlMode.PercentOutput, leftOutput);
            simMotors[2].set(ControlMode.PercentOutput, rightOutput);
        }
    }

    public void setMotorVelocityMetersPerSecond(double leftSpeed, double rightSpeed) {
        m_leftOutput = leftSpeed / Drive.kMaxVelocityMetersPerSecond;
        m_rightOutput = rightSpeed / Drive.kMaxVelocityMetersPerSecond;

        // smartDashboard.putNumber("DriveTrain", "leftOutput", m_leftOutput);
        // smartDashboard.putNumber("DriveTrain", "rightSpeed", m_rightOutput);

        driveMotors[0].set(ControlMode.Velocity, m_leftOutput / (Drive.kEncoderDistancePerPulseMeters * 10), DemandType.ArbitraryFeedForward, feedforward.calculate(m_leftOutput));
        driveMotors[2].set(ControlMode.Velocity, m_rightOutput / (Drive.kEncoderDistancePerPulseMeters * 10), DemandType.ArbitraryFeedForward, feedforward.calculate(m_rightOutput));
    }

    /**
     * Sets drivetrain motors to coast/brake
     *
     * @param mode 2 = all coast, 1 = all brake, 0 = half and half
     */
    public void setDriveTrainNeutralMode(int mode) {
        switch (mode) {
            case 2:
                for (var motor : driveMotors)
                    motor.setNeutralMode(NeutralMode.Coast);
                for (var brakeMode : brakeMode)
                    brakeMode = false;
                break;
            case 1:
                for (var motor : driveMotors)
                    motor.setNeutralMode(NeutralMode.Brake);
                for (var brakeMode : brakeMode)
                    brakeMode = true;
                break;
            case 0:
            default:
                driveMotors[0].setNeutralMode(NeutralMode.Brake);
                driveMotors[1].setNeutralMode(NeutralMode.Coast);
                driveMotors[2].setNeutralMode(NeutralMode.Brake);
                driveMotors[3].setNeutralMode(NeutralMode.Coast);
                brakeMode[0] = true;
                brakeMode[1] = false;
                brakeMode[2] = true;
                brakeMode[3] = false;
                break;
        }
    }

    public DifferentialDriveWheelSpeeds getSpeedsMetersPerSecond() {
        double leftMetersPerSecond = 0, rightMetersPerSecond = 0;

        if (RobotBase.isReal()) {
//             getSelectedSensorVelocity() returns values in units per 100ms. Need to convert value to RPS
            leftMetersPerSecond = driveMotors[0].getSelectedSensorVelocity() * Drive.kEncoderDistancePerPulseMeters * 10.0;
            rightMetersPerSecond = driveMotors[2].getSelectedSensorVelocity() * Drive.kEncoderDistancePerPulseMeters * 10.0;
        } else {
            leftMetersPerSecond = driveMotors[0].getSelectedSensorVelocity() * Drive.kEncoderDistancePerPulseMetersSim * 10.0;
            rightMetersPerSecond = driveMotors[2].getSelectedSensorVelocity() * Drive.kEncoderDistancePerPulseMetersSim * 10.0;
        }
        return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
    }

    public double getTravelDistanceMeters() {
        double leftMeters, rightMeters;

        if (RobotBase.isReal()) {
            leftMeters = (driveMotors[0].getSelectedSensorPosition() * 10.0 / 2048) * gearRatio * Math.PI * Drive.kWheelDiameterMeters;
            rightMeters = (driveMotors[2].getSelectedSensorPosition() * 10.0 / 2048) * gearRatio * Math.PI * Drive.kWheelDiameterMeters;
        } else {
            leftMeters = (simMotors[0].getSelectedSensorPosition() * 10.0 / 4096) * Math.PI * Drive.kWheelDiameterMeters;
            rightMeters = (simMotors[2].getSelectedSensorPosition() * 10.0 / 4096) * Math.PI * Drive.kWheelDiameterMeters;
        }
        return (leftMeters + rightMeters) / 2.0;
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public Pose2d getRobotPose2d() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveKinematics getDriveTrainKinematics() {
        return kinematics;
    }

    public PIDController getLeftPIDController() {
        return leftPIDController;
    }

    public PIDController getRightPIDController() {
        return rightPIDController;
    }

    public void resetOdometry(Pose2d pose, Rotation2d rotation) {
        if (RobotBase.isSimulation()) {
            resetEncoderCounts();
            m_drivetrainSimulator.setPose(pose);
        }

        odometry.resetPosition(Rotation2d.fromDegrees(getHeadingDegrees()), getEncoderCount(0), getEncoderCount(2), getRobotPose2d());
        resetEncoderCounts();
    }

    // private void initShuffleboardValues() {
    //     // Need to verify that this works again
    //     Shuffleboard.getTab("Drive Train").addNumber("Left Encoder", () -> getEncoderCount(0));
    //     Shuffleboard.getTab("Drive Train").addNumber("Right Encoder", () -> getEncoderCount(2));
    //     Shuffleboard.getTab("Drive Train").addNumber("xCoordinate", () ->
    //             Units.metersToFeet(getRobotPose2d().getTranslation().getX()));
    //     Shuffleboard.getTab("Drive Train").addNumber("yCoordinate", () ->
    //             Units.metersToFeet(getRobotPose2d().getTranslation().getY()));
    //     Shuffleboard.getTab("Drive Train").addNumber("Angle", () ->
    //             getRobotPose2d().getRotation().getDegrees());
    //     Shuffleboard.getTab("Drive Train").addNumber("leftSpeed", () ->
    //             Units.metersToFeet(getSpeedsMetersPerSecond().leftMetersPerSecond));
    //     Shuffleboard.getTab("Drive Train").addNumber("rightSpeed", () ->
    //             Units.metersToFeet(getSpeedsMetersPerSecond().rightMetersPerSecond));


    //     Shuffleboard.getTab("Turret").addNumber("Robot Angle", navX::getAngle);
    // }

    private void updateSmartDashboard() {
        // if (RobotBase.isReal()) {
        //     smartDashboard.putNumber("DriveTrain", "Left Distance", getWheelDistanceMeters(0));
        //     smartDashboard.putNumber("DriveTrain", "Right Distance", getWheelDistanceMeters(2));
        //     smartDashboard.putNumber("DriveTrain", "xCoordinate",
        //             Units.metersToFeet(getRobotPose2d().getTranslation().getX()));
        //     smartDashboard.putNumber("DriveTrain", "yCoordinate",
        //             Units.metersToFeet(getRobotPose2d().getTranslation().getY()));
        //     smartDashboard.putNumber("DriveTrain", "Angle", getRobotPose2d().getRotation().getDegrees());
        //     smartDashboard.putNumber("DriveTrain", "leftSpeed",
        //             Units.metersToFeet(getSpeedsMetersPerSecond().leftMetersPerSecond));
        //     smartDashboard.putNumber("DriveTrain", "rightSpeed",
        //             Units.metersToFeet(getSpeedsMetersPerSecond().rightMetersPerSecond));

        //     smartDashboard.putNumber("Turret", "Robot Angle", getHeadingDegrees());

        //     // TODO: Remove this debug code when finished tuning
        //     smartDashboard.putNumber("DriveTrain", "kS", kS);
        //     smartDashboard.putNumber("DriveTrain", "kV", kV);
        //     smartDashboard.putNumber("DriveTrain", "kA", kA);
        // } else {
        //     smartDashboard.putNumber("DriveTrain", "Left Encoder", getEncoderCount(0));
        //     smartDashboard.putNumber("DriveTrain", "Right Encoder", getEncoderCount(2));
        //     smartDashboard.putNumber("DriveTrain", "xCoordinate",
        //             Units.metersToFeet(getRobotPose2d().getTranslation().getX()));
        //     smartDashboard.putNumber("DriveTrain", "yCoordinate",
        //             Units.metersToFeet(getRobotPose2d().getTranslation().getY()));
        //     smartDashboard.putNumber("DriveTrain", "Angle", getRobotPose2d().getRotation().getDegrees());
        //     smartDashboard.putNumber("DriveTrain", "leftSpeed",
        //             Units.metersToFeet(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
        //     smartDashboard.putNumber("DriveTrain", "rightSpeed",
        //             Units.metersToFeet(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));

        //     smartDashboard.putNumber("Turret", "Robot Angle", getHeadingDegrees());
        // }
    }

    // TODO: Remove this debug code when finished tuning
    // Allow these values to be set through SmartDashoard
    private void updateVoltageValues() {
        // kS = smartDashboard.getNumber("DriveTrain", "kS", Drive.ksVolts);
        // kV = smartDashboard.getNumber("DriveTrain", "kV", Drive.kvVoltSecondsPerMeter);
        // kA = smartDashboard.getNumber("DriveTrain", "kA", Drive.kaVoltSecondsSquaredPerMeter);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        odometry.update(Rotation2d.fromDegrees(getHeadingDegrees()), getWheelDistanceMeters(0), getWheelDistanceMeters(2));
        updateVoltageValues();
        updateSmartDashboard();
    }

    public double getDrawnCurrentAmps() {
        return m_drivetrainSimulator.getCurrentDrawAmps();
    }

    @Override
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the simulation,
        // and write the simulated positions and velocities to our simulated encoder and gyro.
        // We negate the right side so that positive voltages make the right side
        // move forward.

        m_drivetrainSimulator.setInputs(m_leftOutput * RobotController.getBatteryVoltage(),
                m_rightOutput * RobotController.getBatteryVoltage());
        m_drivetrainSimulator.update(0.040);

        // For CTRE devices, you must call this function periodically for simulation
        Unmanaged.feedEnable(40);
        simMotors[0].getSimCollection().setQuadratureRawPosition((int) (m_drivetrainSimulator.getLeftPositionMeters() / Drive.kEncoderDistancePerPulseMetersSim));
        simMotors[0].getSimCollection().setQuadratureVelocity((int) (m_drivetrainSimulator.getLeftVelocityMetersPerSecond() / (Drive.kEncoderDistancePerPulseMetersSim * 10.0)));
        simMotors[2].getSimCollection().setQuadratureRawPosition((int) (m_drivetrainSimulator.getRightPositionMeters() / Drive.kEncoderDistancePerPulseMetersSim));
        simMotors[2].getSimCollection().setQuadratureVelocity((int) (m_drivetrainSimulator.getRightVelocityMetersPerSecond() / (Drive.kEncoderDistancePerPulseMetersSim * 10.0)));
        m_gyroAngleSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

        SmartDashboard.putNumber("Robot Angle", getHeadingDegrees());
        SmartDashboard.putNumber("L Encoder Count", simMotors[0].getSelectedSensorPosition());
        SmartDashboard.putNumber("R Encoder Count", simMotors[2].getSelectedSensorPosition());
        SmartDashboard.putNumber("L Encoder Rate", simMotors[0].getSelectedSensorVelocity());
        SmartDashboard.putNumber("R Encoder Rate", simMotors[2].getSelectedSensorVelocity());

        SmartDashboard.putNumber("L Output", m_leftOutput);
        SmartDashboard.putNumber("R Output", m_rightOutput);
        SmartDashboard.putNumber("L Encoder Distance", getWheelDistanceMeters(0));
        SmartDashboard.putNumber("R Encoder Distance", getWheelDistanceMeters(2));
//        SmartDashboard.putNumber("L Encoder Count", m_leftEncoder.get());
//        SmartDashboard.putNumber("R Encoder Count", m_rightEncoder.get());
//        SmartDashboard.putNumber("L Encoder Rate", m_leftEncoder.getRate());
//        SmartDashboard.putNumber("R Encoder Rate", m_rightEncoder.getRate());

        SmartDashboard.putBoolean("CTRE Feed Enabled", Unmanaged.getEnableState());
    }
}