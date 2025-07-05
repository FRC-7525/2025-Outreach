package frc.robot.Subsystems.Indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Subsystems.Indexer.IndexerConstants.MAX_VOLTAGE;
import static frc.robot.Subsystems.Indexer.IndexerConstants.SUBSYSTEM_NAME;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;

public class IndexerIOTalon implements IndexerIO {

	private double motorSpeedSetpoint;
	private TalonFX motor;

	private DigitalInput topSensor = new DigitalInput(IndexerConstants.TOP_SENSOR_PORT);
	private DigitalInput middleSensor = new DigitalInput(IndexerConstants.MIDDLE_SENSOR_PORT);
	private DigitalInput bottomSensor = new DigitalInput(IndexerConstants.BOTTOM_SENSOR_PORT);

	public IndexerIOTalon() {
		motorSpeedSetpoint = 0.0;
		motor = new TalonFX(IndexerConstants.INDEXER_MOTOR_CAN_ID);
	}

	@Override
	public double getMotorSpeed() {
		return motor.getVelocity().getValue().in(RotationsPerSecond);
	}

	@Override
	public int getBallCount() {
		if (topSensor.get() && middleSensor.get() && bottomSensor.get()) return 3;
		if (middleSensor.get() && bottomSensor.get()) return 2;
		if (bottomSensor.get()) return 1;
		return 0;
	}

	@Override
	public double getMotorSpeedSetpoint() {
		return motorSpeedSetpoint;
	}

	@Override
	public void setMotorSpeed(double speed) {
		this.motorSpeedSetpoint = speed;
		motor.setVoltage(speed * MAX_VOLTAGE.in(Volts));
	}

	@Override
	public void logInfo() {
		Logger.recordOutput(SUBSYSTEM_NAME + "/Speed Setpoint", getMotorSpeedSetpoint());
		Logger.recordOutput(SUBSYSTEM_NAME + "/Motor Speed", getMotorSpeed());
		Logger.recordOutput(
			SUBSYSTEM_NAME + "/Input Voltage",
			motor.getMotorVoltage().getValueAsDouble()
		);
		Logger.recordOutput(SUBSYSTEM_NAME + "/Ball Count", getBallCount());
	}

	public void setBallCount(int count) { //only used in sim
		return;
	}
}
