// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotarians;

import java.util.function.Consumer;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class KrakenPidNetworkTableHelper extends BasePidNetworkTableCreator {

    public KrakenPidNetworkTableHelper(String tableName, PidValuesRecord pidValues) {
        super(tableName, "Duty Cycle", pidValues);
    }

    public void dashboardUpdate(
            TalonFX motor,
            Slot0Configs configs,
            Consumer<Double> goToTarget,
            Consumer<Boolean> extraPidUpdate) {
        m_velocityPub.set(motor.getVelocity().getValueAsDouble());
        m_positionPub.set(motor.getPosition().getValueAsDouble());
        m_outputnPub.set(motor.getDutyCycle().getValueAsDouble());

        TryReadPids((values) -> {
            configs.kP = values.kP();
            configs.kI = values.kI();
            configs.kD = values.kD();
            motor.getConfigurator().apply(configs);
            extraPidUpdate.accept(true);
        });

        TryReadTarget((target) -> goToTarget.accept(target));
    }
}
