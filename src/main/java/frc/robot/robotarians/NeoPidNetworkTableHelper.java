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

import edu.wpi.first.networktables.DoublePublisher;

/** Add your docs here. */
public class NeoPidNetworkTableHelper extends BasePidNetworkTableCreator {
    private final DoublePublisher m_appliedOutputPublisher;

    public NeoPidNetworkTableHelper(String tableName,
            double kP, double kI, double kD) {
        super(tableName, kP, kI, kD);
        m_appliedOutputPublisher = m_dataTable.getDoubleTopic("Applied Output").publish();
    }

    public void dashboardUpdate(
            SparkMax motor,
            SparkRelativeEncoder encoder,
            SparkMaxConfig config,
            Consumer<Double> goToTarget) {
        m_velocityPub.set(encoder.getVelocity());
        m_positionPub.set(encoder.getPosition());
        m_appliedOutputPublisher.set(motor.getAppliedOutput());

        if (m_updatePidSub.get()) {
            config.closedLoop
                    .pid(m_kPSub.get(), m_kISub.get(), m_kDSub.get());
            motor.configure(config, ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);

            m_updatePidPub.set(false);
        }

        if (m_goToTargetSub.get()) {
            goToTarget.accept(m_targetSub.get());

            m_goToTargetPub.set(false);
        }
    }
}
