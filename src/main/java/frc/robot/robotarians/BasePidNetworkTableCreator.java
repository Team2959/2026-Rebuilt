// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotarians;

import java.util.function.Consumer;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class BasePidNetworkTableCreator {
    private final DoubleSubscriber m_kPSub;
    private final DoubleSubscriber m_kISub;
    private final DoubleSubscriber m_kDSub;
    private final BooleanSubscriber m_updatePidSub;
    private final BooleanPublisher m_updatePidPub;

    private final DoubleSubscriber m_targetSub;
    private final BooleanSubscriber m_goToTargetSub;
    private final BooleanPublisher m_goToTargetPub;

    protected final DoublePublisher m_velocityPub;
    protected final DoublePublisher m_positionPub;
    protected final DoublePublisher m_outputnPub;

    private final NetworkTable m_dataTable;

    protected BasePidNetworkTableCreator(String tableName, String outputName, PidValuesRecord pidValues) {
        // get the subtable called "tablename"
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        var datatable = inst.getTable(tableName);
        m_dataTable = datatable;

        // PID topic
        var kpTopic = datatable.getDoubleTopic("kP");
        kpTopic.publish().set(pidValues.kP());
        m_kPSub = kpTopic.subscribe(pidValues.kP());
        var kiTopic = datatable.getDoubleTopic("kI");
        kiTopic.publish().set(pidValues.kI());
        m_kISub = kiTopic.subscribe(pidValues.kI());
        var kDTopic = datatable.getDoubleTopic("kD");
        kDTopic.publish().set(pidValues.kD());
        m_kDSub = kDTopic.subscribe(pidValues.kD());

        var topTopic = datatable.getDoubleTopic("Target");
        topTopic.publish().set(0);
        m_targetSub = topTopic.subscribe(0);

        var goToTarget = datatable.getBooleanTopic("go to Target");
        m_goToTargetPub = goToTarget.publish();
        m_goToTargetPub.set(false);
        m_goToTargetSub = goToTarget.subscribe(false);

        var updatePID = datatable.getBooleanTopic("update PID");
        m_updatePidPub = updatePID.publish();
        m_updatePidPub.set(false);
        m_updatePidSub = updatePID.subscribe(false);

        m_velocityPub = datatable.getDoubleTopic("Velocity").publish();
        m_positionPub = datatable.getDoubleTopic("Position").publish();
        m_outputnPub = datatable.getDoubleTopic(outputName).publish();
    }

    protected void TryReadPids(Consumer<PidValuesRecord> updatePidValues) {
        if (m_updatePidSub.get()) {
            updatePidValues.accept(new PidValuesRecord(m_kPSub.get(), m_kISub.get(), m_kDSub.get()));

            m_updatePidPub.set(false);
        }
    }

    protected void TryReadTarget(Consumer<Double> updateTarget) {
        if (m_goToTargetSub.get()) {
            updateTarget.accept(m_targetSub.get());

            m_goToTargetPub.set(false);
        }
    }

    public NetworkTable networkTable() {
        return m_dataTable;
    }
}
