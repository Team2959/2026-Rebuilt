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
public class BasePidNetworkTableCreator {
    protected final DoubleSubscriber m_kPSub;
    protected final DoubleSubscriber m_kISub;
    protected final DoubleSubscriber m_kDSub;
    protected final DoubleSubscriber m_kFfSub;
    protected final BooleanSubscriber m_updatePidSub;
    protected final BooleanPublisher m_updatePidPub;

    protected final DoubleSubscriber m_targetSub;
    protected final BooleanSubscriber m_goToTargetSub;
    protected final BooleanPublisher m_goToTargetPub;

    protected final DoublePublisher m_velocityPub;
    protected final DoublePublisher m_positionPub;

    protected final NetworkTable m_dataTable;

    protected BasePidNetworkTableCreator(String tableName, double kP, double kI, double kD){
        // get the subtable called "tablename"
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        var datatable = inst.getTable(tableName);
        m_dataTable = datatable;

        // PID topic
        var kpTopic = datatable.getDoubleTopic("kP");
        kpTopic.publish().set(kP);
        m_kPSub = kpTopic.subscribe(kP);
        var kiTopic = datatable.getDoubleTopic("kI");
        kiTopic.publish().set(kI);
        m_kISub = kiTopic.subscribe(kI);
        var kDTopic = datatable.getDoubleTopic("kD");
        kDTopic.publish().set(kD);
        m_kDSub = kDTopic.subscribe(kD);
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
    }
}
