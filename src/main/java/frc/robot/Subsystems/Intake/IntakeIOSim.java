package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.SIMULATION_PERIOD;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {

	private TalonFX pivotMotor;
	private TalonFX wheelMotor;
	private TalonFXSimState pivotMotorTalonSim;
	private TalonFXSimState wheelMotorTalonSim;

	private SingleJointedArmSim pivotSim;
	private DCMotorSim wheelMotorSim;
	private PIDController pivotController;
	private PIDController wheelSpeedController;

	private double wheelSpeedSetpoint;
	private double pivotPositionSetpoint;

	private double lastIntookSimulatedGamepieceTime;

	IntakeIOSim() {
		wheelMotor = new TalonFX(IntakeConstants.Real.WHEEL_MOTOR_CANID);
		pivotMotor = new TalonFX(IntakeConstants.Real.PIVOT_MOTOR_CANID);
		wheelMotorTalonSim = wheelMotor.getSimState();
		pivotMotorTalonSim = pivotMotor.getSimState();

		pivotSim = new SingleJointedArmSim(
			DCMotor.getKrakenX60(IntakeConstants.Sim.NUM_PIVOT_MOTORS),
			IntakeConstants.Sim.PIVOT_GEARING,
			IntakeConstants.Sim.PIVOT_MOI.in(KilogramSquareMeters),
			IntakeConstants.Sim.PIVOT_ARM_LENGTH.in(Meters),
			IntakeConstants.Sim.MIN_PIVOT_ANGLE.in(Radians),
			IntakeConstants.Sim.MAX_PIVOT_ANGLE.in(Radians),
			false,
			IntakeConstants.Sim.STARTING_PIVOT_ANGLE.in(Radians)
		);

		wheelMotorSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getKrakenX60(IntakeConstants.Sim.NUM_WHEEL_MOTORS),
				IntakeConstants.Sim.WHEEL_MOTOR_MOI.in(KilogramSquareMeters),
				IntakeConstants.Sim.WHEEL_MOTOR_GEARING
			),
			DCMotor.getKrakenX60(IntakeConstants.Sim.NUM_WHEEL_MOTORS)
		);

		pivotController = IntakeConstants.PIVOT_PID_CONTROLELR.get();
		wheelSpeedController = IntakeConstants.VELOCITY_PID_CONTROLELR.get();

		wheelSpeedSetpoint = 0;
		pivotPositionSetpoint = 0;
		lastIntookSimulatedGamepieceTime = 0;
	}

	@Override
	public void updateInputs(IntakeIOInputs input) {
		pivotSim.update(SIMULATION_PERIOD);
		wheelMotorSim.update(SIMULATION_PERIOD);

		input.wheelSpeed = Units.radiansToDegrees(wheelMotorSim.getAngularVelocityRadPerSec());
		input.wheelSpeedSetpoint = wheelSpeedSetpoint;
		input.pivotPosition = Units.radiansToDegrees(pivotSim.getAngleRads());
		input.pivotSetpoint = pivotPositionSetpoint;

		pivotMotorTalonSim.setRawRotorPosition(pivotSim.getAngleRads());
		pivotMotorTalonSim.setRotorVelocity(
			Units.radiansToRotations(pivotSim.getVelocityRadPerSec())
		);

		wheelMotorTalonSim.setRawRotorPosition(wheelMotorSim.getAngularPosition());
		wheelMotorTalonSim.setRotorVelocity(wheelMotorSim.getAngularVelocity());
	}

	@Override
	public void setPivotSetpoint(double pivotSetpoint) {
		this.pivotPositionSetpoint = pivotSetpoint;
		pivotSim.setInputVoltage(
			pivotController.calculate(
				Units.radiansToDegrees(pivotSim.getAngleRads()),
				pivotSetpoint
			)
		);
	}

	@Override
	public void setWheelSpeed(double wheelSpeedSetpoint) {
		this.wheelSpeedSetpoint = wheelSpeedSetpoint;
		wheelMotorSim.setInputVoltage(
			wheelSpeedController.calculate(
				wheelMotorSim.getAngularVelocityRadPerSec(),
				wheelSpeedSetpoint
			)
		);
	}

	public boolean hasGamepiece() {
		boolean gamepieceInIntake =
			Intake.getInstance().getStateTime() - lastIntookSimulatedGamepieceTime >
			IntakeConstants.Sim.SIMULATED_INTAKING_TIME.in(Seconds);
		if (gamepieceInIntake) {
			lastIntookSimulatedGamepieceTime = Intake.getInstance().getStateTime();
		}
		return gamepieceInIntake;
	}
}
