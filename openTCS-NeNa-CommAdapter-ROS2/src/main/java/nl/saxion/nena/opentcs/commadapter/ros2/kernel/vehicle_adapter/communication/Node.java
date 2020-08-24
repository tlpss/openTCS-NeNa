package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication;

import action_msgs.msg.GoalStatusArray;
import geometry_msgs.msg.PoseStamped;
import std_msgs.msg.Bool;
import geometry_msgs.msg.PoseWithCovarianceStamped;
import lombok.Getter;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.ComposableNode;
import org.ros2.rcljava.publisher.Publisher;

import javax.annotation.Nonnull;

/**
 * Class that holds an an instances of a node and of all its publishers and subscriptions.
 * {@link org.ros2.rcljava.node.BaseComposableNode} does not support namespaces,
 * which is the reason for a the development of the Node class.
 *
 * @author Niels Tiben
 */
@Getter
public class Node implements ComposableNode {
    private final org.ros2.rcljava.node.Node node;
    private final Publisher<PoseWithCovarianceStamped> initialPosePublisher;
    private final Publisher<PoseStamped> goalPublisher;
    private final Publisher<Bool> startDockingPublisher;

    public Node(@Nonnull NodeMessageListener nodeMessageListener, @Nonnull java.lang.String namespace) {
        this.node = RCLJava.createNode("opentcs", namespace, RCLJava.getDefaultContext());

        /* --------------- Publishers ---------------*/
        // Publisher for setting the initial pose
        this.initialPosePublisher = node.createPublisher(PoseWithCovarianceStamped.class, namespace + "/initialpose");

        // Publisher for sending a navigation goal
        this.goalPublisher = node.createPublisher(PoseStamped.class, namespace + "/move_base_simple/goal");

        /* --------------- Subscriptions ---------------*/
        // Subscriber for navigation status
        node.createSubscription(GoalStatusArray.class, namespace + "/NavigateToPose/_action/status",
                nodeMessageListener::onNewGoalStatusArray);

        // Subscription for current location
        node.createSubscription(PoseWithCovarianceStamped.class, namespace + "/amcl_pose",
                nodeMessageListener::onNewAmclPose);

        // TODO: Implement Subscription for battery data (TurtleBot3 specific)

        // TODO: Implement publisher and subscriber for LOAD CARGO operation.

        // TODO: Implement publisher and subscriber for UNLOAD CARGO operation.
        this.startDockingPublisher = node.createPublisher(Bool.class, namespace + "/start_docking");
    }
}