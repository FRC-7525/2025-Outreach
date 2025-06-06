package frc.robot.Subsystems.Indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.Indexer.IndexerConstants.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Subsystems.Indexer.IndexerConstants.SIM;

public class IndexerIOSim implements IndexerIO {

    private double motorSpeedSetpoint = 0.0;
    private int ballCount = 0;

    private TalonFX dummyMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR_CAN_ID);
    private TalonFXSimState motorSim = dummyMotor.getSimState();

    private FlywheelSim flywheelSim = new FlywheelSim(SIM.MOTOR_LINEAR_SYSTEM, DCMotor.getKrakenX60Foc(INDEXER_MOTOR_CAN_ID), null);

    @Override
    public double getMotorSpeed() {
        motorSim.setRotorVelocity(flywheelSim.getAngularVelocity());
        return flywheelSim.getAngularVelocity().in(RotationsPerSecond);

    }

    @Override
    public double getMotorSpeedSetpoint() {
        return motorSpeedSetpoint;
    }

    @Override
    public int getBallCount() {
        return ballCount;
    }

    @Override
    public void setMotorSpeed(double speed) {
        
    }

    @Override
    public void logInfo() {
        Logger.recordOutput(SUBSYSTEM_NAME +"/Speed Setpoint", getMotorSpeedSetpoint());
        Logger.recordOutput(SUBSYSTEM_NAME +"/Motor Speed", getMotorSpeed());
        Logger.recordOutput(SUBSYSTEM_NAME + "/Ball Count", getBallCount());
    }
    
}
