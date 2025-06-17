package frc.robot.Subsystems.AdjustableHood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.GlobalConstants.SIMULATION_PERIOD;
import static frc.robot.Subsystems.AdjustableHood.AdjustableHoodConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class AdjustableHoodIOSim implements AdjustableHoodIO {

	private TalonFX pivotMotor;
	private TalonFXSimState pivotMotorTalonSim;
	private SingleJointedArmSim pivotSim;
	private ProfiledPIDController pivotController;
	private Angle pivotSetpoint;

	public AdjustableHoodIOSim() {
		pivotMotor = new TalonFX(Real.PIVOT_CAN_ID);
		pivotMotorTalonSim = pivotMotor.getSimState();
		pivotSim = new SingleJointedArmSim(
			DCMotor.getKrakenX60(Sim.NUM_PIVOT_MOTORS),
			Sim.GEAR_RATIO,
			Sim.PIVOT_MOI.in(KilogramSquareMeters),
			Sim.PIVOT_LENGTH.in(Meters),
			MIN_ANGLE.in(Radians),
			MAX_ANGLE.in(Radians),
			false,
			Sim.PIVOT_STARTING_ANGLE.in(Radians)
		);

		pivotController = PIVOT_PID.get();
		pivotController.setTolerance(Sim.PIVOT_TOLERANCE.in(Radians));
		pivotSetpoint = Sim.PIVOT_STARTING_ANGLE;
	}

	@Override
	public void updateInputs(AdjustableHoodIOInputs inputs) {
		pivotSim.update(SIMULATION_PERIOD);

		inputs.hoodSetpoint = pivotSetpoint;
		inputs.hoodPosition = Units.radiansToDegrees(pivotSim.getAngleRads());

		pivotMotorTalonSim.setRawRotorPosition(Units.radiansToRotations(pivotSim.getAngleRads()));
		pivotMotorTalonSim.setRotorVelocity(
			Units.radiansToRotations(pivotSim.getVelocityRadPerSec())
		);
	}

	@Override
	public void setHoodSetpoint(Angle setpoint) {
		pivotSetpoint = setpoint;
		pivotSim.setInputVoltage(
			pivotController.calculate(
				Units.radiansToDegrees(pivotSim.getAngleRads()),
				pivotSetpoint.in(Degrees)
			)
		);
	}

	@Override
	public boolean atHoodSetpoint() {
		return pivotController.atSetpoint();
	}
}