package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.GlobalConstants.RobotMode;

public class IntakeIOReal implements IntakeIO {

	private TalonFX wheelMotor;
	private TalonFX pivotMotor;
	private PIDController pivotController;
	private PIDController wheelSpeedController;
	private double pivotPositionSetpoint;
	private double wheelSpeedSetpoint;
	private DigitalInput beamBreak;

	public IntakeIOReal() {
		wheelMotor = new TalonFX(IntakeConstants.Real.WHEEL_MOTOR_CANID);
		pivotMotor = new TalonFX(IntakeConstants.Real.PIVOT_MOTOR_CANID);
		wheelMotor.setPosition(0);
		pivotMotor.setPosition(0);
		pivotController = new PIDController(
			IntakeConstants.Real.PIVOT_PID_CONSTANTS.kP,
			IntakeConstants.Real.PIVOT_PID_CONSTANTS.kI,
			IntakeConstants.Real.PIVOT_PID_CONSTANTS.kD
		);
		wheelSpeedController = new PIDController(
			IntakeConstants.Real.WHEEL_PID_CONSTANTS.kP,
			IntakeConstants.Real.WHEEL_PID_CONSTANTS.kI,
			IntakeConstants.Real.WHEEL_PID_CONSTANTS.kD
		);
		beamBreak = new DigitalInput(IntakeConstants.Real.BEAM_BREAK_DIO_PORT);
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		inputs.pivotPosition = Units.rotationsToDegrees(
			pivotMotor.getPosition().getValueAsDouble()
		);
		inputs.pivotSetpoint = pivotPositionSetpoint;
		inputs.wheelSpeed = wheelMotor.getVelocity().getValueAsDouble();
		inputs.wheelSpeedSetpoint = wheelSpeedSetpoint;

		if (GlobalConstants.ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData("Intake Pivot PID", pivotController);
			SmartDashboard.putData("Intake Wheel Speed PID", wheelSpeedController);
		}
	}

	@Override
	public void setPivotSetpoint(double pivotSetpoint) {
		this.pivotPositionSetpoint = pivotSetpoint;
		double voltage = pivotController.calculate(
			Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble()),
			pivotSetpoint
		);
		pivotMotor.setVoltage(voltage);
	}

	@Override
	public void setWheelSpeed(double wheelSpeed) {
		this.wheelSpeedSetpoint = wheelSpeed;
		double voltage = wheelSpeedController.calculate(
			Units.rotationsToDegrees(wheelMotor.getVelocity().getValueAsDouble()),
			wheelSpeed
		);
		wheelMotor.setVoltage(voltage);
	}

	@Override
	public boolean hasGamepiece() {
		return beamBreak.get();
	}
}
