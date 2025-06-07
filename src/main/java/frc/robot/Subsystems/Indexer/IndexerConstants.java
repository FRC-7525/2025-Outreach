package frc.robot.Subsystems.Indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;

public class IndexerConstants {

	public static final String SUBSYSTEM_NAME = "Indexer";
	public static final int NUM_MOTOR = 1;
	public static final int INDEXER_MOTOR_CAN_ID = 15;

	public static final int TOP_SENSOR_PORT = 0; // random values
	public static final int MIDDLE_SENSOR_PORT = 1;
	public static final int BOTTOM_SENSOR_PORT = 2;

	public static final double IDLE_SPEED = 0.0;
	public static final double SHOOTING_SPEED = 1;
	public static final double INTAKING_SPEED = -.5;

	public static final Voltage MAX_VOLTAGE = Volts.of(12.0);

	public static class SIM {

		public static final LinearSystem<N1, N1, N1> MOTOR_LINEAR_SYSTEM =
			LinearSystemId.createFlywheelSystem(
				DCMotor.getKrakenX60Foc(NUM_MOTOR), // random values
				1,
				1
			);
	}
}
