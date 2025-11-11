package frc.robot.Subsystems.Indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.GlobalConstants.SIMULATION_PERIOD;
import static frc.robot.Subsystems.Indexer.IndexerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Indexer.IndexerConstants.SIM;

public class IndexerIOSim implements IndexerIO {

	private double motorSpeedSetpoint = 0.0;
	private int ballCount = 0;

	private TalonFX dummyMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR_CAN_ID);
	private TalonFXSimState motorSim = dummyMotor.getSimState();

	private double[] measurementStdDevs = { SIM.STDDEV };

	private FlywheelSim flywheelSim = new FlywheelSim(
		SIM.MOTOR_LINEAR_SYSTEM,
		DCMotor.getKrakenX60Foc(INDEXER_MOTOR_CAN_ID),
		measurementStdDevs
	);

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
		motorSpeedSetpoint = speed;
		flywheelSim.setInputVoltage(speed * MAX_VOLTAGE.in(Volts));
	}

	@Override
	public void logInfo() {
		flywheelSim.update(SIMULATION_PERIOD);

		SmartDashboard.putNumber(SUBSYSTEM_NAME + "/Speed Setpoint", getMotorSpeedSetpoint());
		SmartDashboard.putNumber(SUBSYSTEM_NAME + "/Motor Speed", getMotorSpeed());
		SmartDashboard.putNumber(SUBSYSTEM_NAME + "/Input Voltage", flywheelSim.getInputVoltage());
		SmartDashboard.putNumber(SUBSYSTEM_NAME + "/Ball Count", getBallCount());
	}

	@Override
	public void setBallCount(int count) {
		this.ballCount = count;
	}
}
