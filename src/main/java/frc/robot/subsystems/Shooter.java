// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.unmanaged.Unmanaged;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.Shooter.kDriveSimEncoderDistancePerPulse;
import static frc.robot.Constants.Shooter.kFlywheelGearRatio;

public class Shooter extends SubsystemBase {

    private final TalonFX[] flywheelMotors = {
        new TalonFX(Constants.CAN.flywheelMotorA),
        new TalonFX(Constants.CAN.flywheelMotorB),
    };
    private TalonSRX m_flywheelMotorSim;

    private final Vision m_vision;
    private double rpmSetpoint;

    private final LinearSystem<N1, N1, N1> m_flywheelPlant = LinearSystemId.identifyVelocitySystem(Constants.Shooter.kV, Constants.Shooter.kA);

    private final KalmanFilter<N1, N1, N1> m_observer =
            new KalmanFilter<>(
                    Nat.N1(),
                    Nat.N1(),
                    m_flywheelPlant,
                    VecBuilder.fill(3.0), // How accurate we think our model is
                    VecBuilder.fill(0.01), // How accurate we think our encoder
                    // data is
                    0.020);

    private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
            new LinearQuadraticRegulator<>(
                    m_flywheelPlant,
                    VecBuilder.fill(8.0), // Velocity error tolerance
                    VecBuilder.fill(12.0), // Control effort (voltage) tolerance
                    0.020);

    private final LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

    private final FlywheelSim m_flywheelSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(Constants.Shooter.kV, Constants.Shooter.kA),
            DCMotor.getFalcon500(2),
            kFlywheelGearRatio);

    private double m_percentOutput;
    private double m_simEncoderDistance;

    public Shooter(Vision vision) {
        for(TalonFX flywheelMotor : flywheelMotors) {
            flywheelMotor.configFactoryDefault();
            flywheelMotor.setNeutralMode(NeutralMode.Coast);
            flywheelMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
            flywheelMotor.configVoltageCompSaturation(10);
            flywheelMotor.enableVoltageCompensation(true);
        }
        flywheelMotors[0].setInverted(true);
        flywheelMotors[1].follow(flywheelMotors[0], FollowerType.PercentOutput);

        m_vision = vision;

        if(RobotBase.isSimulation()) {
            m_flywheelMotorSim = new TalonSRX(flywheelMotors[0].getDeviceID());

            m_flywheelMotorSim.configFactoryDefault();
            m_flywheelMotorSim.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        }
    }

    // TODO: Need a way to determine if it can shoot
    public boolean canShoot() {
        return false;
    }

    public void setPercentOutput(double percentOutput) {
        m_percentOutput = percentOutput;
        flywheelMotors[0].set(ControlMode.PercentOutput, m_percentOutput);
    }

    public void setRpmSetpoint(double setpoint) {
        this.rpmSetpoint = setpoint;
    }

    public double getRpm(int index){
        if(RobotBase.isReal())
            return falconUnitsToRPM(flywheelMotors[index].getSelectedSensorVelocity());
        else
            return m_flywheelMotorSim.getSelectedSensorVelocity() / 4096 * 10 * 60;
    }

    public double getRpmSetpoint() {
        return rpmSetpoint;
    }

    public double falconUnitsToRPM(double sensorUnits) {
        return (sensorUnits / 2048.0) * 600.0;
    }

    public double RPMtoFalconUnits(double RPM) {
        return (RPM / 600.0) * 2048.0;
    }

    private void updateRPMSetpoint() {
        if(rpmSetpoint > 0) {
            m_loop.setNextR(VecBuilder.fill(rpmSetpoint));

            m_loop.correct(VecBuilder.fill(getRpm(0)));

            m_loop.predict(0.020);

            double nextVoltage = m_loop.getU(0);

            setPercentOutput(nextVoltage / 12.0);
        }
    }

    @Override
    public void periodic() {
        updateRPMSetpoint();

        SmartDashboard.putNumber("Shooter RPM", getRpm(0));
    }

    @Override
    public void simulationPeriodic() {
        m_flywheelSim.setInputVoltage(m_percentOutput * RobotController.getBatteryVoltage());

        m_flywheelSim.update(0.02);

        m_simEncoderDistance += m_flywheelSim.getAngularVelocityRadPerSec() * 0.02 / (Math.PI * 2) * 4096;

        Unmanaged.feedEnable(20);
        m_flywheelMotorSim.getSimCollection().setQuadratureRawPosition((int) (m_simEncoderDistance));
        m_flywheelMotorSim.getSimCollection().setQuadratureVelocity((int) (m_flywheelSim.getAngularVelocityRadPerSec() / (Math.PI * 2) * 4096 / 10.0));
    }
}