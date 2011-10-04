package proteus;

import com.google.common.base.Preconditions;

import org.apache.commons.logging.Log;
import org.ros.DefaultNode;
import org.ros.MessageListener;
import org.ros.Node;
import org.ros.NodeConfiguration;
import org.ros.NodeMain;
import org.ros.Publisher;
import org.ros.Subscriber;

import org.ros.message.geometry_msgs.Twist;
import org.ros.message.sensor_msgs.LaserScan;

public class AvoidObstacleLaser implements NodeMain {

	private Node node;

	@Override
	public void main(NodeConfiguration configuration) {
		Preconditions.checkState(node == null);
		Preconditions.checkNotNull(configuration);
		try {
			node = new DefaultNode("laser_cmd", configuration);
			final Log log = node.getLog();
			log.info("init");
			final Publisher<Twist> publisher = 
				node.createPublisher("/ATRV/Motion_Controller", "geometry_msgs/Twist");
			node.createSubscriber("/ATRV/Sick", "sensor_msgs/LaserScan", 
				new MessageListener<LaserScan>() {
				@Override
				public void onNewMessage(LaserScan msg) {
					int i, mid = msg.ranges.length / 2;
					Twist cmd = new Twist();
					// halt if an object is less than 2m in a 30deg angle
					boolean halt = false;
					for (i = mid - 15; i <= mid + 15; i++) {
						if (msg.ranges[i] < 2.0) {
							halt = true;
							break;
						}
					}
					if (halt) {
						double midA = 0, midB = 0;
						// we go to the highest-range side scanned
						for (i = 0; i < mid; i++)
							midA += msg.ranges[i];
						for (i = mid; i < msg.ranges.length; i++)
							midB += msg.ranges[i];
						log.info("A:" + midA + ", B:" + midB);
						if (midA > midB) {
							cmd.angular.z = -1;
						} else {
							cmd.angular.z = 1;
						}
					} else {
						cmd.linear.x = 1;
					}
					publisher.publish(cmd);
				}
			});
		} catch (Exception e) {
			if (node != null) {
				node.getLog().fatal(e);
			} else {
				e.printStackTrace();
			}
		}
	}

	@Override
	public void shutdown() {
		node.shutdown();
		node = null;
	}

}

