package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;
import edu.wpi.first.units.measure.AngularVelocity;
import static frc.robot.GlobalConstants.*;

public class Shooter extends Subsystem<ShooterStates> {
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs;
    private AngularVelocity dynamicSetpoint;
    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null) {
            ShooterIO shooterIO = 
                switch (ROBOT_MODE) {
                    case SIM -> new ShooterIOSim();
                    case REAL -> new ShooterIOReal();
                    case TESTING -> new ShooterIOReal();
                };
            instance = new Shooter(shooterIO);
        }

        return instance;
    }

    private Shooter(ShooterIO io) {
        super("Shooter", ShooterStates.IDLE);
        this.io = io;
        inputs = new ShooterIOInputsAutoLogged();
    }

    @Override
    public void runState() {
        if (getState() == ShooterStates.DYNAMIC) {
            io.setShooterSetpoint(dynamicSetpoint);
        } else {
            io.setShooterSetpoint(getState().getSetpoint());
        }

        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/State", getState().getStateString());
    }

    public void setDynamicShooterSetpoint(AngularVelocity setpoint) {
        dynamicSetpoint = setpoint;
    }

    public boolean atSetpoint() {
        return io.atTargetSpeed();
    }
}
