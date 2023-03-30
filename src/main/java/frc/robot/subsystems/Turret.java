/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DIO;
import frc.robot.Constants.SimConstants;

/**
 * Subsystem for controlling the turret
 */
public class Turret extends SubsystemBase {
    private final DriveTrain m_driveTrain;
    private final CANCoder encoder = new CANCoder(CAN.turretEncoder);
    private final VictorSPX turretMotor = new VictorSPX(CAN.turretMotor);
    private final DigitalInput turretHomeSensor = new DigitalInput(DIO.turretHome);

    // setup variables

    // Turret PID gains
    double kF = 0.07;
    double kP = 0.2;
    double kI = 0.0015;
    double kD = 0.0;
    int kI_Zone = 900;
    int kMaxIAccum = 1000000;
    int kErrorBand = 50;
    int kCruiseVelocity = 10000;
    int kMotionAcceleration = kCruiseVelocity * 10;
    double minAngleDegrees = -90;
    double maxAngleDegrees = 90;
    double gearRatio = 18.0 / 120.0;
    
    private double setpoint = 0; //angle
    private boolean initialHome;
    private boolean usingSensor = false;
    private boolean turretHomeSensorLatch = false;

    /**
     * Creates a new Turret.
     */
    public Turret(DriveTrain driveTrain) {
        // Setup turret motors
        m_driveTrain = driveTrain;
        encoder.configFactoryDefault();
        encoder.setPositionToAbsolute();
        encoder.configSensorDirection(true);

        turretMotor.configFactoryDefault();
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.setInverted(true);
        turretMotor.configRemoteFeedbackFilter(61, RemoteSensorSource.CANCoder, 0, 0);
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        turretMotor.config_kF(0, Constants.Turret.kF);
        turretMotor.config_kP(0, Constants.Turret.kP);
        turretMotor.config_kI(0, Constants.Turret.kI);
        turretMotor.config_IntegralZone(0, Constants.Turret.kI_Zone);
        turretMotor.configMaxIntegralAccumulator(0, Constants.Turret.kMaxIAccum);
        turretMotor.config_kD(0, Constants.Turret.kD);
        turretMotor.configMotionCruiseVelocity(Constants.Turret.kCruiseVelocity);
        turretMotor.configMotionAcceleration(Constants.Turret.kMotionAcceleration);
        turretMotor.configAllowableClosedloopError(0, Constants.Turret.kErrorBand);
    }

    public void resetEncoder() {
        turretMotor.setSelectedSensorPosition(0);
        encoder.setPosition(0);
    }

    public boolean getUsingSensor() {
        return usingSensor;
    }

    public void setUsingSensor(boolean using) {
        usingSensor = using;
    }

    public double getTurretAngleDegrees() {
        return encoderUnitsToDegrees(turretMotor.getSelectedSensorPosition());
    }

    public double getFieldRelativeAngleDegrees() {
        return getTurretAngleDegrees() - m_driveTrain.getHeadingDegrees();
    }

    public double getMaxAngleDegrees() {
        return Constants.Turret.maxAngleDegrees;
    }

    public double getMinAngleDegrees() {
        return Constants.Turret.minAngleDegrees;
    }
    public boolean getTurretHome() {
        return !turretHomeSensor.get();
    }

    /**
     * Checks if the robot is in its starting position
     */
    public boolean getInitialHome() {
        return initialHome;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setPercentOutput(double output) {
        turretMotor.set(ControlMode.PercentOutput, output);
    }

    public void setRobotCentricSetpointDegrees(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setFieldCentricSetpointDegrees(double setpoint) {
        setpoint -= m_driveTrain.getHeadingDegrees();

        if(setpoint > getMaxAngleDegrees())
            setpoint -= 360;
        else if(setpoint < getMinAngleDegrees())
            setpoint += 360;

        this.setpoint = setpoint;
    }

    public void setClosedLoopPositionDegrees() {
        turretMotor.set(ControlMode.MotionMagic, degreesToEncoderUnits(getSetpoint()));
    }

    public int degreesToEncoderUnits(double degrees) {
        return (int) (degrees * (1.0 / Constants.Turret.gearRatio) * (Constants.Turret.encoderUnitsPerRotation / 360.0));
    }

    public double encoderUnitsToDegrees(double encoderUnits) {
        return encoderUnits * Constants.Turret.gearRatio * (360.0 / Constants.Turret.encoderUnitsPerRotation);
    }

    /**
     * checks if the turret is pointing within the tolerance of the target
     */
    public boolean onTarget() {
        return Math.abs(turretMotor.getClosedLoopError()) < Constants.Turret.kErrorBand;
    }

    public void clearIAccum() {
        turretMotor.setIntegralAccumulator(0);
    }

    private void setTurretLatch(boolean state) {
        turretHomeSensorLatch = state;
    }

    public boolean getTurretLatch() {
        return turretHomeSensorLatch;
    }

    public void stopTurret() {
        setpoint = getTurretAngleDegrees();
    }

    private void initShuffleboard() {
        // Unstable. Don''t use until WPILib fixes this
        Shuffleboard.getTab("Turret").addNumber("Turret Motor Output", turretMotor :: getMotorOutputPercent);
        Shuffleboard.getTab("Turret").addNumber("Turret Robot Relative Angle", this :: getTurretAngleDegrees);
        Shuffleboard.getTab("Turret").addNumber("Turret Field Relative Angle", this :: getFieldRelativeAngleDegrees);
        Shuffleboard.getTab("Turret").addNumber("Turret Setpoint", this :: getSetpoint);
        Shuffleboard.getTab("Turret").addNumber("Turret Error", turretMotor :: getClosedLoopError);
        Shuffleboard.getTab("Turret").addNumber("Turret IAccum", turretMotor :: getIntegralAccumulator);
        Shuffleboard.getTab("Turret").addBoolean("Home", this :: getTurretHome);
    }

    // set smartdashboard
    private void updateSmartdashboard() {
        if (RobotBase.isReal()) {
            SmartDashboard.putNumber("Turret Angle", getFieldRelativeAngleDegrees());

            SmartDashboard.putNumber(/*"Turret",*/ "Turret Motor Output", turretMotor.getMotorOutputPercent());
            SmartDashboard.putNumber(/*"Turret",*/ "Turret Robot Relative Angle", getTurretAngleDegrees());
            SmartDashboard.putNumber(/*"Turret",*/ "Turret Field Relative Angle", getFieldRelativeAngleDegrees());
            SmartDashboard.putNumber(/*"Turret",*/ "Turret Setpoint", getSetpoint());
            SmartDashboard.putBoolean(/*"Turret",*/ "Home", getTurretHome());
        }
    }

    @Override
    public void periodic() {
        if(getUsingSensor())
            setClosedLoopPositionDegrees();

        // This method will be called once per scheduler run
        // TODO: FIX
        // Fix what??
        if(! getTurretLatch() && getTurretHome()) {
            turretMotor.setSelectedSensorPosition(0);
            encoder.setPosition(0);
            setTurretLatch(true);
        } else if(getTurretLatch() && ! getTurretHome())
            setTurretLatch(false);

        if(! initialHome)
            if(getTurretHome())
                initialHome = true;

        updateSmartdashboard();
    }

    public double getTurretSimAngleDegrees() {
        return getTurretAngleDegrees() + 180;
    }

    public Pose2d getTurretSimPoseMeters() {
        return new Pose2d(m_driveTrain.getRobotPose2d().getX(),
                          m_driveTrain.getRobotPose2d().getY(),
                          new Rotation2d(Math.toRadians(getTurretSimAngleDegrees())));
    }


    @Override
    public void simulationPeriodic() {
    }

    public double getIdealTargetDistanceMeters() {
        return Math.sqrt(Math.pow(SimConstants.blueGoalPoseMeters.getY() - getTurretSimPoseMeters().getY(), 2) + Math.pow(SimConstants.blueGoalPoseMeters.getX() - getTurretSimPoseMeters().getX(), 2));
    }

    public double getIdealTurretAngle() {

        double targetRadians = Math.atan2(SimConstants.blueGoalPoseMeters.getY() - getTurretSimPoseMeters().getY(), SimConstants.blueGoalPoseMeters.getX() - getTurretSimPoseMeters().getX());

        return Math.toDegrees(targetRadians);
    }
}
