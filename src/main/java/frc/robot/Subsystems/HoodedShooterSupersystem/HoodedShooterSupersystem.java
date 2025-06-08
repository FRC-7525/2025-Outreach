package frc.robot.Subsystems.HoodedShooterSupersystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Subsystems.HoodedShooterSupersystem.HoodedShooterSupersystemConstants.SUBSYSTEM_NAME;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Subsystems.AdjustableHood.AdjustableHood;
import frc.robot.Subsystems.Shooter.Shooter;

public class HoodedShooterSupersystem extends Subsystem<HoodedShooterSupersystemStates> {
    private static HoodedShooterSupersystem instance;
    private AdjustableHood hood;
    private Shooter shooter;

    public static HoodedShooterSupersystem getInstance() {
        if (instance == null) {
            instance = new HoodedShooterSupersystem();
        }
        return instance;
    }

    private HoodedShooterSupersystem() {
        super("HoodedShooterSupersystem", HoodedShooterSupersystemStates.IDLE);
        hood = AdjustableHood.getInstance();
        shooter = Shooter.getInstance();
    }

    @Override
    public void runState() {
        if (getState() == HoodedShooterSupersystemStates.DYNAMIC) {
            hood.setDynamicHoodSetpoint(calculateHoodSetpoint());
            shooter.setDynamicShooterSetpoint(calculateShooterSetpoint());
        }
        
        hood.setState(getState().getAdjustableHoodState());
        shooter.setState(getState().getShooterState());

        hood.periodic();
        shooter.periodic();

        Logger.recordOutput(SUBSYSTEM_NAME + "/State", getState().getStateString());
    }


    /*
     * Checks if the hood and shooter are both at their setpoints.
     * The manager subsystem should use this method to determine when to switch from warming up to shooting.
     * Those states should be identical but one tells the indexer to pass the ball to the shooter.
     */
    public boolean readyToShoot() {
        return hood.atSetpoint() && shooter.atSetpoint();
    }
    
    private Angle calculateHoodSetpoint() {
        // Placeholder for actual hood setpoint calculation logic
        return Degrees.of(15); // Replace with actual calculation
    }

    private AngularVelocity calculateShooterSetpoint() {
        // Placeholder for actual shooter setpoint calculation logic
        return RotationsPerSecond.of(100); // Replace with actual calculation
    }
}
