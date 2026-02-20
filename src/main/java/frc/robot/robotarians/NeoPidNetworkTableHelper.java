// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotarians;

import java.util.function.Consumer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public class NeoPidNetworkTableHelper extends BasePidNetworkTableCreator {

    public NeoPidNetworkTableHelper(String tableName, PidValuesRecord pidValues) {
        super(tableName, "Applied Output", pidValues);
    }

    public void dashboardUpdate(
            SparkMax motor,
            SparkRelativeEncoder encoder,
            SparkMaxConfig config,
            Consumer<Double> goToTarget,
            Consumer<Boolean> extraPidUpdate) {
        m_velocityPub.set(encoder.getVelocity());
        m_positionPub.set(encoder.getPosition());
        m_outputnPub.set(motor.getAppliedOutput());

        TryReadPids((values) -> {
            config.closedLoop
                    .pid(values.kP(), values.kI(), values.kD());
            extraPidUpdate.accept(true);
            motor.configure(config, ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
        });

        TryReadTarget((target) -> goToTarget.accept(target));
    }
}
