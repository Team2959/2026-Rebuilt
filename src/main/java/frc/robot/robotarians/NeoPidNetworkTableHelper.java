// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotarians;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class NeoPidNetworkTableHelper {
    private final DoubleSubscriber m_kPSub;
    private final DoubleSubscriber m_kISub;
    private final DoubleSubscriber m_kDSub;
    private final DoubleSubscriber m_kFfSub;
    private final BooleanSubscriber m_updatePidSub;
    private final BooleanPublisher m_updatePidPub;

    private final DoubleSubscriber m_targetSub;
    private final BooleanSubscriber m_goToTargetSub;
    private final BooleanPublisher m_goToTargetPub;

    private final DoublePublisher m_velocityPub;
    private final DoublePublisher m_positionPub;
    private final DoublePublisher m_appliedOutputPublisher;

    public NeoPidNetworkTableHelper(String tableName) {
        // get the subtable called "serveMod1"
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable datatable = inst.getTable(tableName);

        // PID topic
        var kpTopic = datatable.getDoubleTopic("kP");
        kpTopic.publish().set(0);
        m_kPSub = kpTopic.subscribe(0);
        var kiTopic = datatable.getDoubleTopic("kI");
        kiTopic.publish().set(0);
        m_kISub = kiTopic.subscribe(0);
        var kDTopic = datatable.getDoubleTopic("kD");
        kDTopic.publish().set(0);
        m_kDSub = kDTopic.subscribe(0);
        var ffTopic = datatable.getDoubleTopic("FF");
        ffTopic.publish().set(0);
        m_kFfSub = ffTopic.subscribe(0);

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
        m_appliedOutputPublisher = datatable.getDoubleTopic("Applied Output").publish();
    }
}
