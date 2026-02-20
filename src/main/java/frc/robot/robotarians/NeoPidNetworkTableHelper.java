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

    public NeoPidNetworkTableHelper(String tableName,
            double kP, double kI, double kD) {
        super(tableName, "Applied Output", kP, kI, kD);
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
            extraPidUpdate.accept(true);
            config.closedLoop
                    .pid(values.kP(), values.kI(), values.kD());
            motor.configure(config, ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
        });

        TryReadTarget((target) -> goToTarget.accept(target));
    }
}
